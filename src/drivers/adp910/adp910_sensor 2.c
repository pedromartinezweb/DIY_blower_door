#include "drivers/adp910/adp910_sensor.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stddef.h>

#define ADP910_CMD_START_CONTINUOUS 0x361Eu
#define ADP910_SAMPLE_FRAME_SIZE 6u
#define ADP910_STARTUP_DELAY_MS 60u
#define ADP910_FIRST_SAMPLE_DELAY_MS 20u
#define ADP910_STABILIZATION_SAMPLE_COUNT 3u
#define ADP910_STABILIZATION_DELAY_MS 10u

static uint8_t adp910_crc8(const uint8_t *data, uint8_t length) {
  uint8_t crc = 0xFFu;
  uint8_t byte_index = 0u;

  for (byte_index = 0u; byte_index < length; ++byte_index) {
    uint8_t bit_index = 0u;
    crc ^= data[byte_index];

    for (bit_index = 0u; bit_index < 8u; ++bit_index) {
      crc = (crc & 0x80u) != 0u ? (uint8_t)((crc << 1u) ^ 0x31u)
                                : (uint8_t)(crc << 1u);
    }
  }

  return crc;
}

static adp910_status_t adp910_write_command(const adp910_sensor_t *sensor,
                                            uint16_t command) {
  uint8_t command_bytes[2];
  int bytes_written = 0;

  command_bytes[0] = (uint8_t)(command >> 8u);
  command_bytes[1] = (uint8_t)(command & 0xFFu);

  bytes_written = i2c_write_blocking(sensor->port_config.i2c_instance,
                                     sensor->port_config.i2c_address,
                                     command_bytes, sizeof(command_bytes),
                                     false);

  return bytes_written == (int)sizeof(command_bytes) ? ADP910_STATUS_OK
                                                      : ADP910_STATUS_BUS_ERROR;
}

static adp910_status_t adp910_read_raw_frame(const adp910_sensor_t *sensor,
                                             uint8_t *raw_frame,
                                             size_t frame_length) {
  int bytes_read = 0;

  bytes_read = i2c_read_blocking(sensor->port_config.i2c_instance,
                                 sensor->port_config.i2c_address, raw_frame,
                                 frame_length, false);

  return bytes_read == (int)frame_length ? ADP910_STATUS_OK
                                          : ADP910_STATUS_BUS_ERROR;
}

adp910_status_t adp910_sensor_start_continuous_mode(adp910_sensor_t *sensor) {
  if (sensor == NULL || sensor->port_config.i2c_instance == NULL) {
    return ADP910_STATUS_INVALID_ARGUMENT;
  }

  return adp910_write_command(sensor, ADP910_CMD_START_CONTINUOUS);
}

adp910_status_t adp910_sensor_initialize(
    adp910_sensor_t *sensor, const adp910_port_config_t *port_config) {
  uint8_t sample_index = 0u;

  if (sensor == NULL || port_config == NULL || port_config->i2c_instance == NULL) {
    return ADP910_STATUS_INVALID_ARGUMENT;
  }

  *sensor = (adp910_sensor_t){
      .port_config = *port_config,
      .pressure_offset_pa = 0.0f,
      .is_initialized = false,
  };

  i2c_init(sensor->port_config.i2c_instance, sensor->port_config.i2c_frequency_hz);

  gpio_set_function(sensor->port_config.sda_pin, GPIO_FUNC_I2C);
  gpio_set_function(sensor->port_config.scl_pin, GPIO_FUNC_I2C);
  gpio_pull_up(sensor->port_config.sda_pin);
  gpio_pull_up(sensor->port_config.scl_pin);

  sleep_ms(ADP910_STARTUP_DELAY_MS);

  if (adp910_sensor_start_continuous_mode(sensor) != ADP910_STATUS_OK) {
    return ADP910_STATUS_BUS_ERROR;
  }

  sleep_ms(ADP910_FIRST_SAMPLE_DELAY_MS);
  sensor->is_initialized = true;

  for (sample_index = 0u; sample_index < ADP910_STABILIZATION_SAMPLE_COUNT;
       ++sample_index) {
    adp910_sample_t discarded_sample;
    (void)adp910_sensor_read_sample(sensor, &discarded_sample);
    sleep_ms(ADP910_STABILIZATION_DELAY_MS);
  }

  return ADP910_STATUS_OK;
}

adp910_status_t adp910_sensor_read_sample(adp910_sensor_t *sensor,
                                          adp910_sample_t *out_sample) {
  uint8_t raw_frame[ADP910_SAMPLE_FRAME_SIZE];
  int16_t raw_pressure = 0;
  int16_t raw_temperature = 0;

  if (sensor == NULL || out_sample == NULL) {
    return ADP910_STATUS_INVALID_ARGUMENT;
  }

  if (!sensor->is_initialized) {
    return ADP910_STATUS_NOT_READY;
  }

  if (adp910_read_raw_frame(sensor, raw_frame, sizeof(raw_frame)) !=
      ADP910_STATUS_OK) {
    return ADP910_STATUS_BUS_ERROR;
  }

  if (adp910_crc8(raw_frame, 2u) != raw_frame[2] ||
      adp910_crc8(raw_frame + 3u, 2u) != raw_frame[5]) {
    return ADP910_STATUS_CRC_MISMATCH;
  }

  raw_pressure = (int16_t)(((uint16_t)raw_frame[0] << 8u) | raw_frame[1]);
  raw_temperature = (int16_t)(((uint16_t)raw_frame[3] << 8u) | raw_frame[4]);

  out_sample->differential_pressure_pa = (float)raw_pressure / 60.0f;
  out_sample->corrected_pressure_pa =
      out_sample->differential_pressure_pa - sensor->pressure_offset_pa;
  out_sample->temperature_c = (float)raw_temperature / 200.0f;

  return ADP910_STATUS_OK;
}

void adp910_sensor_set_pressure_offset(adp910_sensor_t *sensor,
                                       float pressure_offset_pa) {
  if (sensor == NULL) {
    return;
  }

  sensor->pressure_offset_pa = pressure_offset_pa;
}

float adp910_sensor_get_pressure_offset(const adp910_sensor_t *sensor) {
  if (sensor == NULL) {
    return 0.0f;
  }

  return sensor->pressure_offset_pa;
}
