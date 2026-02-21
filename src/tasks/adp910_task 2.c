#include "tasks/task_entries.h"

#include "app/app_config.h"
#include "drivers/adp910/adp910_sensor.h"
#include "services/blower_metrics.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define ADP910_INIT_RETRY_BACKOFF_MS 1000u

static const blower_linear_fan_speed_model_config_t
    k_fan_speed_model_config = {
        .pascal_to_speed_gain = APP_FAN_PRESSURE_TO_SPEED_GAIN,
    };

static const blower_linear_air_leakage_model_config_t
    k_air_leakage_model_config = {
        .leakage_gain = APP_AIR_LEAKAGE_GAIN,
    };

static const adp910_port_config_t k_fan_sensor_port_config = {
    .i2c_instance = APP_ADP910_FAN_SENSOR_I2C_INSTANCE,
    .i2c_address = APP_ADP910_I2C_ADDRESS,
    .sda_pin = APP_ADP910_FAN_SENSOR_SDA_PIN,
    .scl_pin = APP_ADP910_FAN_SENSOR_SCL_PIN,
    .i2c_frequency_hz = APP_ADP910_I2C_FREQUENCY_HZ,
};

static const adp910_port_config_t k_envelope_sensor_port_config = {
    .i2c_instance = APP_ADP910_ENVELOPE_SENSOR_I2C_INSTANCE,
    .i2c_address = APP_ADP910_I2C_ADDRESS,
    .sda_pin = APP_ADP910_ENVELOPE_SENSOR_SDA_PIN,
    .scl_pin = APP_ADP910_ENVELOPE_SENSOR_SCL_PIN,
    .i2c_frequency_hz = APP_ADP910_I2C_FREQUENCY_HZ,
};

static bool adp910_try_initialize_sensor(const char *sensor_label,
                                         adp910_sensor_t *sensor,
                                         const adp910_port_config_t *port_config) {
  const adp910_status_t status = adp910_sensor_initialize(sensor, port_config);

  if (status == ADP910_STATUS_OK) {
    (void)sensor_label;
    (void)port_config;
    return true;
  }

  (void)sensor_label;
  return false;
}

void adp910_sampling_task_entry(void *params) {
  adp910_sensor_t fan_sensor = {0};
  adp910_sensor_t envelope_sensor = {0};
  bool fan_sensor_ready = false;
  bool envelope_sensor_ready = false;
  TickType_t fan_retry_allowed_tick = 0;
  TickType_t envelope_retry_allowed_tick = 0;
  TickType_t next_wake_tick = xTaskGetTickCount();
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
  (void)params;

  while (1) {
    adp910_sample_t fan_sample = {0};
    adp910_sample_t envelope_sample = {0};
    bool fan_sample_valid = false;
    bool envelope_sample_valid = false;

    if (!fan_sensor_ready && xTaskGetTickCount() >= fan_retry_allowed_tick) {
      fan_sensor_ready = adp910_try_initialize_sensor(
          "Fan sensor", &fan_sensor, &k_fan_sensor_port_config);
      if (!fan_sensor_ready) {
        fan_retry_allowed_tick =
            xTaskGetTickCount() + pdMS_TO_TICKS(ADP910_INIT_RETRY_BACKOFF_MS);
      }
    }

    if (!envelope_sensor_ready &&
        xTaskGetTickCount() >= envelope_retry_allowed_tick) {
      envelope_sensor_ready = adp910_try_initialize_sensor(
          "Envelope sensor", &envelope_sensor, &k_envelope_sensor_port_config);
      if (!envelope_sensor_ready) {
        envelope_retry_allowed_tick =
            xTaskGetTickCount() + pdMS_TO_TICKS(ADP910_INIT_RETRY_BACKOFF_MS);
      }
    }

    if (fan_sensor_ready) {
      const adp910_status_t fan_read_status =
          adp910_sensor_read_sample(&fan_sensor, &fan_sample);
      fan_sample_valid = fan_read_status == ADP910_STATUS_OK;

      if (fan_read_status == ADP910_STATUS_BUS_ERROR ||
          fan_read_status == ADP910_STATUS_NOT_READY) {
        fan_sensor_ready = false;
      }
    }

    if (envelope_sensor_ready) {
      const adp910_status_t envelope_read_status =
          adp910_sensor_read_sample(&envelope_sensor, &envelope_sample);
      envelope_sample_valid = envelope_read_status == ADP910_STATUS_OK;

      if (envelope_read_status == ADP910_STATUS_BUS_ERROR ||
          envelope_read_status == ADP910_STATUS_NOT_READY) {
        envelope_sensor_ready = false;
      }
    }

    blower_metrics_service_update(fan_sample_valid ? &fan_sample : NULL,
                                  fan_sample_valid,
                                  envelope_sample_valid ? &envelope_sample
                                                        : NULL,
                                  envelope_sample_valid);

#if APP_ADP910_LOG_EVERY_N_CYCLES > 0
    loop_counter += 1u;
    if (loop_counter >= APP_ADP910_LOG_EVERY_N_CYCLES) {
      blower_metrics_snapshot_t snapshot;
      loop_counter = 0u;

      if (blower_metrics_service_get_snapshot(&snapshot)) {
        printf("[ADP910] fan_dp=%.3f Pa env_dp=%.3f Pa speed=%.3f leakage=%.3f\n",
               snapshot.fan_pressure_pa, snapshot.envelope_pressure_pa,
               snapshot.fan_speed_units, snapshot.estimated_air_leakage_units);
      }
    }
#endif

    vTaskDelayUntil(&next_wake_tick,
                    pdMS_TO_TICKS(APP_ADP910_SAMPLE_PERIOD_MS));
  }
}
