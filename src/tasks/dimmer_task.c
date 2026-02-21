#include "tasks/task_entries.h"

#include "app/app_config.h"
#include "FreeRTOS.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "services/blower_control.h"
#include "services/blower_metrics.h"
#include "services/dimmer_control.h"
#include "task.h"
#include <math.h>
#include <stdint.h>

#define DIMMER_GATE_PULSE_US 100u
#define DIMMER_FREQUENCY_DOUBLE_EDGE_THRESHOLD_HZ 70.0f

static volatile uint32_t g_last_zero_cross_us = 0u;
static volatile uint32_t g_zero_cross_period_us = 0u;

static bool dimmer_pick_control_pressure(
    const blower_metrics_snapshot_t *snapshot, float *out_pressure_pa) {
  if (snapshot == NULL || out_pressure_pa == NULL) {
    return false;
  }

#if APP_CONTROL_PRESSURE_SOURCE_MODE == APP_CONTROL_PRESSURE_SOURCE_FAN
  if (snapshot->fan_sample_valid) {
    *out_pressure_pa = snapshot->fan_pressure_pa;
    return true;
  }
  return false;
#elif APP_CONTROL_PRESSURE_SOURCE_MODE == APP_CONTROL_PRESSURE_SOURCE_AUTO_MIN_ABS
  if (snapshot->fan_sample_valid && snapshot->envelope_sample_valid) {
    if (fabsf(snapshot->fan_pressure_pa) <=
        fabsf(snapshot->envelope_pressure_pa)) {
      *out_pressure_pa = snapshot->fan_pressure_pa;
    } else {
      *out_pressure_pa = snapshot->envelope_pressure_pa;
    }
    return true;
  }
  if (snapshot->envelope_sample_valid) {
    *out_pressure_pa = snapshot->envelope_pressure_pa;
    return true;
  }
  if (snapshot->fan_sample_valid) {
    *out_pressure_pa = snapshot->fan_pressure_pa;
    return true;
  }
  return false;
#else
  if (snapshot->envelope_sample_valid) {
    *out_pressure_pa = snapshot->envelope_pressure_pa;
    return true;
  }
  return false;
#endif
}

static int64_t dimmer_gate_pulse_alarm_callback(alarm_id_t alarm_id,
                                                void *user_data) {
  gpio_put(APP_DIMMER_GATE_PIN, 1);
  busy_wait_us(DIMMER_GATE_PULSE_US);
  gpio_put(APP_DIMMER_GATE_PIN, 0);
  (void)alarm_id;
  (void)user_data;
  return 0;
}

static void dimmer_zero_crossing_callback(uint gpio, uint32_t events) {
  const uint32_t now_us = time_us_32();
  const uint8_t power_percent = dimmer_control_get_power_percent();
  (void)events;

  if (gpio != APP_DIMMER_ZERO_CROSS_PIN) {
    return;
  }

  if (g_last_zero_cross_us != 0u) {
    g_zero_cross_period_us = now_us - g_last_zero_cross_us;
  }
  g_last_zero_cross_us = now_us;

  if (power_percent > 0u && power_percent < 100u) {
    const uint32_t delay_us = (uint32_t)(100u - power_percent) * 100u;
    add_alarm_in_us(delay_us, dimmer_gate_pulse_alarm_callback, NULL, false);
  } else if (power_percent >= 100u) {
    gpio_put(APP_DIMMER_GATE_PIN, 1);
  } else {
    gpio_put(APP_DIMMER_GATE_PIN, 0);
  }
}

static void dimmer_update_line_feedback(void) {
  uint32_t irq_state = save_and_disable_interrupts();
  const uint32_t last_zero_cross_us = g_last_zero_cross_us;
  const uint32_t period_us = g_zero_cross_period_us;
  restore_interrupts(irq_state);

  const uint32_t now_us = time_us_32();
  const bool line_sync_available =
      last_zero_cross_us != 0u &&
      (now_us - last_zero_cross_us) <= APP_LINE_SYNC_TIMEOUT_US;

  float line_frequency_hz = 0.0f;
  if (line_sync_available && period_us > 0u) {
    line_frequency_hz = 1000000.0f / (float)period_us;
    if (line_frequency_hz > DIMMER_FREQUENCY_DOUBLE_EDGE_THRESHOLD_HZ) {
      line_frequency_hz *= 0.5f;
    }
  }

  blower_control_update_line_feedback(line_sync_available, line_frequency_hz);
}

void dimmer_task_entry(void *params) {
  TickType_t next_wake_tick = xTaskGetTickCount();
  (void)params;

  blower_control_initialize();
  dimmer_control_set_power_percent(0u);

  gpio_init(APP_DIMMER_ZERO_CROSS_PIN);
  gpio_set_dir(APP_DIMMER_ZERO_CROSS_PIN, GPIO_IN);
  gpio_pull_up(APP_DIMMER_ZERO_CROSS_PIN);

  gpio_init(APP_DIMMER_GATE_PIN);
  gpio_set_dir(APP_DIMMER_GATE_PIN, GPIO_OUT);
  gpio_put(APP_DIMMER_GATE_PIN, 0);

  gpio_set_irq_enabled_with_callback(APP_DIMMER_ZERO_CROSS_PIN, GPIO_IRQ_EDGE_RISE,
                                     true, &dimmer_zero_crossing_callback);

  while (1) {
    blower_metrics_snapshot_t metrics_snapshot = {0};
    float control_pressure_pa = 0.0f;
    const uint32_t now_ms =
        (uint32_t)xTaskGetTickCount() * (uint32_t)portTICK_PERIOD_MS;
    const bool has_snapshot =
        blower_metrics_service_get_snapshot(&metrics_snapshot);
    const bool control_pressure_valid =
        has_snapshot &&
        dimmer_pick_control_pressure(&metrics_snapshot, &control_pressure_pa);
    const uint8_t control_output_percent = blower_control_step(
        control_pressure_valid ? control_pressure_pa : 0.0f,
        control_pressure_valid, now_ms);

    dimmer_control_set_power_percent(control_output_percent);
    dimmer_update_line_feedback();

    vTaskDelayUntil(&next_wake_tick,
                    pdMS_TO_TICKS(APP_CONTROL_LOOP_PERIOD_MS));
  }
}
