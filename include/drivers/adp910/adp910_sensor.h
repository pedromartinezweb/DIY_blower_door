#ifndef ADP910_SENSOR_H
#define ADP910_SENSOR_H

#include "hardware/i2c.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
  ADP910_STATUS_OK = 0,
  ADP910_STATUS_INVALID_ARGUMENT,
  ADP910_STATUS_BUS_ERROR,
  ADP910_STATUS_NOT_READY,
  ADP910_STATUS_CRC_MISMATCH
} adp910_status_t;

typedef struct {
  i2c_inst_t *i2c_instance;
  uint8_t i2c_address;
  uint sda_pin;
  uint scl_pin;
  uint32_t i2c_frequency_hz;
} adp910_port_config_t;

typedef struct {
  float differential_pressure_pa;
  float corrected_pressure_pa;
  float temperature_c;
} adp910_sample_t;

typedef struct {
  adp910_port_config_t port_config;
  float pressure_offset_pa;
  bool is_initialized;
  int last_bus_result;
} adp910_sensor_t;

adp910_status_t adp910_sensor_initialize(adp910_sensor_t *sensor,
                                         const adp910_port_config_t *port_config);
adp910_status_t adp910_sensor_start_continuous_mode(adp910_sensor_t *sensor);
adp910_status_t adp910_sensor_read_sample(adp910_sensor_t *sensor,
                                          adp910_sample_t *out_sample);
void adp910_sensor_set_pressure_offset(adp910_sensor_t *sensor,
                                       float pressure_offset_pa);
float adp910_sensor_get_pressure_offset(const adp910_sensor_t *sensor);
int adp910_sensor_get_last_bus_result(const adp910_sensor_t *sensor);

#endif
