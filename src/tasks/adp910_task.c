#include "tasks/task_entries.h"

#include "app/app_config.h"
#include "drivers/adp910/adp910_sensor.h"
#include "services/blower_metrics.h"
#include "FreeRTOS.h"
#include "hardware/gpio.h"
#include "task.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define ADP910_CHANNEL_COUNT 2u
#define ADP910_INIT_RETRY_BACKOFF_MS 1000u
#define ADP910_READ_ERROR_STREAK_TO_REINIT 3u

static const blower_linear_fan_speed_model_config_t
    k_fan_speed_model_config = {
        .pascal_to_speed_gain = APP_FAN_PRESSURE_TO_SPEED_GAIN,
    };

static const blower_linear_air_leakage_model_config_t
    k_air_leakage_model_config = {
        .leakage_gain = APP_AIR_LEAKAGE_GAIN,
    };

typedef struct {
  uint32_t ok;
  uint32_t invalid_argument;
  uint32_t bus_error;
  uint32_t not_ready;
  uint32_t crc_mismatch;
  uint32_t other;
  adp910_status_t last_status;
} adp910_diag_t;

typedef struct {
  const char *id;
  adp910_port_config_t port;
  adp910_sensor_t sensor;
  adp910_diag_t diag;
  bool ready;
  TickType_t next_init_tick;
  adp910_sample_t sample;
  bool sample_valid;
  adp910_status_t last_read_status;
  uint8_t read_error_streak;
} adp910_channel_t;

static const char *adp910_status_name(adp910_status_t status) {
  switch (status) {
  case ADP910_STATUS_OK:
    return "ok";
  case ADP910_STATUS_INVALID_ARGUMENT:
    return "invalid_argument";
  case ADP910_STATUS_BUS_ERROR:
    return "bus_error";
  case ADP910_STATUS_NOT_READY:
    return "not_ready";
  case ADP910_STATUS_CRC_MISMATCH:
    return "crc_mismatch";
  default:
    return "unknown";
  }
}

static uint32_t adp910_i2c_index(const i2c_inst_t *instance) {
  return instance == i2c1 ? 1u : 0u;
}

static void adp910_diag_reset(adp910_diag_t *diag) {
  if (diag == NULL) {
    return;
  }

  *diag = (adp910_diag_t){
      .ok = 0u,
      .invalid_argument = 0u,
      .bus_error = 0u,
      .not_ready = 0u,
      .crc_mismatch = 0u,
      .other = 0u,
      .last_status = ADP910_STATUS_OK,
  };
}

static void adp910_diag_record(adp910_diag_t *diag, adp910_status_t status) {
  if (diag == NULL) {
    return;
  }

  diag->last_status = status;

  switch (status) {
  case ADP910_STATUS_OK:
    diag->ok += 1u;
    break;
  case ADP910_STATUS_INVALID_ARGUMENT:
    diag->invalid_argument += 1u;
    break;
  case ADP910_STATUS_BUS_ERROR:
    diag->bus_error += 1u;
    break;
  case ADP910_STATUS_NOT_READY:
    diag->not_ready += 1u;
    break;
  case ADP910_STATUS_CRC_MISMATCH:
    diag->crc_mismatch += 1u;
    break;
  default:
    diag->other += 1u;
    break;
  }
}

static void adp910_channel_reset_cycle(adp910_channel_t *channel) {
  if (channel == NULL) {
    return;
  }

  channel->sample = (adp910_sample_t){0};
  channel->sample_valid = false;
  channel->last_read_status = ADP910_STATUS_NOT_READY;
}

static void adp910_channel_try_init(adp910_channel_t *channel,
                                    TickType_t now_tick) {
  adp910_status_t init_status = ADP910_STATUS_INVALID_ARGUMENT;

  if (channel == NULL || channel->ready || now_tick < channel->next_init_tick) {
    return;
  }

  init_status = adp910_sensor_initialize(&channel->sensor, &channel->port);
  channel->ready = init_status == ADP910_STATUS_OK;
  adp910_diag_record(&channel->diag, init_status);

  if (!channel->ready) {
    channel->read_error_streak = 0u;
    printf(
        "[ADP910][%s] init_fail status=%s bus=%lu sda=%u sda_lv=%u scl=%u scl_lv=%u addr=0x%02x hz=%lu io=%d\n",
        channel->id, adp910_status_name(init_status),
        (unsigned long)adp910_i2c_index(channel->port.i2c_instance),
        channel->port.sda_pin, gpio_get(channel->port.sda_pin) ? 1u : 0u,
        channel->port.scl_pin, gpio_get(channel->port.scl_pin) ? 1u : 0u,
        (unsigned int)channel->port.i2c_address,
        (unsigned long)channel->port.i2c_frequency_hz,
        adp910_sensor_get_last_bus_result(&channel->sensor));
    channel->next_init_tick =
        now_tick + pdMS_TO_TICKS(ADP910_INIT_RETRY_BACKOFF_MS);
    return;
  }

  printf("[ADP910][%s] init_ok bus=%lu sda=%u scl=%u addr=0x%02x hz=%lu\n",
         channel->id, (unsigned long)adp910_i2c_index(channel->port.i2c_instance),
         channel->port.sda_pin, channel->port.scl_pin,
         (unsigned int)channel->port.i2c_address,
         (unsigned long)channel->port.i2c_frequency_hz);
  channel->read_error_streak = 0u;
}

static void adp910_channel_read(adp910_channel_t *channel) {
  if (channel == NULL || !channel->ready) {
    return;
  }

  channel->last_read_status =
      adp910_sensor_read_sample(&channel->sensor, &channel->sample);
  adp910_diag_record(&channel->diag, channel->last_read_status);
  channel->sample_valid = channel->last_read_status == ADP910_STATUS_OK;

  if (channel->last_read_status == ADP910_STATUS_OK) {
    channel->read_error_streak = 0u;
    return;
  }

  if (channel->last_read_status == ADP910_STATUS_BUS_ERROR ||
      channel->last_read_status == ADP910_STATUS_NOT_READY) {
    if (channel->read_error_streak < 255u) {
      channel->read_error_streak += 1u;
    }
    printf("[ADP910][%s] read_fail status=%s streak=%u sda=%u sda_lv=%u scl=%u scl_lv=%u io=%d\n",
           channel->id,
           adp910_status_name(channel->last_read_status),
           (unsigned int)channel->read_error_streak,
           channel->port.sda_pin, gpio_get(channel->port.sda_pin) ? 1u : 0u,
           channel->port.scl_pin, gpio_get(channel->port.scl_pin) ? 1u : 0u,
           adp910_sensor_get_last_bus_result(&channel->sensor));
    if (channel->read_error_streak >= ADP910_READ_ERROR_STREAK_TO_REINIT) {
      channel->ready = false;
      channel->read_error_streak = 0u;
    }
  }
}

void adp910_sampling_task_entry(void *params) {
  adp910_channel_t channels[ADP910_CHANNEL_COUNT] = {
      {
          .id = "sensor0",
          .port =
              {
                  .i2c_instance = APP_ADP910_FAN_SENSOR_I2C_INSTANCE,
                  .i2c_address = APP_ADP910_FAN_SENSOR_I2C_ADDRESS,
                  .sda_pin = APP_ADP910_FAN_SENSOR_SDA_PIN,
                  .scl_pin = APP_ADP910_FAN_SENSOR_SCL_PIN,
                  .i2c_frequency_hz = APP_ADP910_FAN_SENSOR_I2C_FREQUENCY_HZ,
              },
          .sensor = {0},
          .diag = {0},
          .ready = false,
          .next_init_tick = 0,
          .sample = {0},
          .sample_valid = false,
          .last_read_status = ADP910_STATUS_NOT_READY,
          .read_error_streak = 0u,
      },
      {
          .id = "sensor1",
          .port =
              {
                  .i2c_instance = APP_ADP910_ENVELOPE_SENSOR_I2C_INSTANCE,
                  .i2c_address = APP_ADP910_ENVELOPE_SENSOR_I2C_ADDRESS,
                  .sda_pin = APP_ADP910_ENVELOPE_SENSOR_SDA_PIN,
                  .scl_pin = APP_ADP910_ENVELOPE_SENSOR_SCL_PIN,
                  .i2c_frequency_hz = APP_ADP910_ENVELOPE_SENSOR_I2C_FREQUENCY_HZ,
              },
          .sensor = {0},
          .diag = {0},
          .ready = false,
          .next_init_tick = 0,
          .sample = {0},
          .sample_valid = false,
          .last_read_status = ADP910_STATUS_NOT_READY,
          .read_error_streak = 0u,
      },
  };
  TickType_t next_wake_tick = xTaskGetTickCount();
  size_t index = 0u;
#if APP_ADP910_LOG_EVERY_N_CYCLES > 0
  uint32_t loop_counter = 0u;
#endif

  const blower_metrics_models_t models = {
      .fan_speed_model = blower_linear_fan_speed_model,
      .fan_speed_model_context = &k_fan_speed_model_config,
      .air_leakage_model = blower_linear_air_leakage_model,
      .air_leakage_model_context = &k_air_leakage_model_config,
  };

  blower_metrics_service_initialize(&models);
  for (index = 0u; index < ADP910_CHANNEL_COUNT; ++index) {
    adp910_diag_reset(&channels[index].diag);
  }
  (void)params;

  while (1) {
    const TickType_t now_tick = xTaskGetTickCount();
    adp910_channel_t *channel0 = &channels[0];
    adp910_channel_t *channel1 = &channels[1];

    for (index = 0u; index < ADP910_CHANNEL_COUNT; ++index) {
      adp910_channel_reset_cycle(&channels[index]);
    }
    for (index = 0u; index < ADP910_CHANNEL_COUNT; ++index) {
      adp910_channel_try_init(&channels[index], now_tick);
    }
    for (index = 0u; index < ADP910_CHANNEL_COUNT; ++index) {
      adp910_channel_read(&channels[index]);
    }

    blower_metrics_service_update(
        channel0->sample_valid ? &channel0->sample : NULL, channel0->sample_valid,
        channel1->sample_valid ? &channel1->sample : NULL, channel1->sample_valid);

#if APP_ADP910_LOG_EVERY_N_CYCLES > 0
    loop_counter += 1u;
    if (loop_counter >= APP_ADP910_LOG_EVERY_N_CYCLES) {
      blower_metrics_snapshot_t snapshot;
      loop_counter = 0u;

      if (blower_metrics_service_get_snapshot(&snapshot)) {
        printf("[ADP910][diag] seq=%lu s0_ready=%u s0_last=%s s0_ok=%lu s0_bus=%lu s0_crc=%lu s0_nr=%lu s1_ready=%u s1_last=%s s1_ok=%lu s1_bus=%lu s1_crc=%lu s1_nr=%lu s0_dp=%.3f s1_dp=%.3f\n",
               (unsigned long)snapshot.update_sequence,
               channel0->ready ? 1u : 0u,
               adp910_status_name(channel0->diag.last_status),
               (unsigned long)channel0->diag.ok,
               (unsigned long)channel0->diag.bus_error,
               (unsigned long)channel0->diag.crc_mismatch,
               (unsigned long)channel0->diag.not_ready,
               channel1->ready ? 1u : 0u,
               adp910_status_name(channel1->diag.last_status),
               (unsigned long)channel1->diag.ok,
               (unsigned long)channel1->diag.bus_error,
               (unsigned long)channel1->diag.crc_mismatch,
               (unsigned long)channel1->diag.not_ready,
               snapshot.fan_pressure_pa, snapshot.envelope_pressure_pa);
      }
    }
#endif

    vTaskDelayUntil(&next_wake_tick,
                    pdMS_TO_TICKS(APP_ADP910_SAMPLE_PERIOD_MS));
  }
}
