#ifndef BLOWER_METRICS_H
#define BLOWER_METRICS_H

#include "drivers/adp910/adp910_sensor.h"
#include <stdbool.h>
#include <stdint.h>

typedef float (*blower_fan_speed_model_fn)(float fan_pressure_pa,
                                           const void *context);
typedef float (*blower_air_leakage_model_fn)(float fan_speed_units,
                                             float envelope_pressure_pa,
                                             const void *context);

typedef struct {
  blower_fan_speed_model_fn fan_speed_model;
  const void *fan_speed_model_context;
  blower_air_leakage_model_fn air_leakage_model;
  const void *air_leakage_model_context;
} blower_metrics_models_t;

typedef struct {
  float pascal_to_speed_gain;
} blower_linear_fan_speed_model_config_t;

typedef struct {
  float leakage_gain;
} blower_linear_air_leakage_model_config_t;

typedef enum {
  BLOWER_CAL_NONE = 0,
  BLOWER_CAL_SAMPLING = 1,
  BLOWER_CAL_DONE = 2,
} blower_calibration_state_t;

typedef struct {
  float fan_pressure_pa;
  float fan_temperature_c;
  float envelope_pressure_pa;
  float envelope_temperature_c;
  float fan_speed_units;
  float estimated_air_leakage_units;
  bool fan_sample_valid;
  bool envelope_sample_valid;
  uint32_t update_sequence;
  uint32_t last_update_tick;
  blower_calibration_state_t calibration_state;
  uint8_t calibration_progress_pct;
  float calibration_fan_offset;
  float calibration_envelope_offset;
} blower_metrics_snapshot_t;

void blower_metrics_service_initialize(const blower_metrics_models_t *models);
void blower_metrics_service_update(const adp910_sample_t *fan_sample,
                                   bool fan_sample_valid,
                                   const adp910_sample_t *envelope_sample,
                                   bool envelope_sample_valid);
bool blower_metrics_service_get_snapshot(blower_metrics_snapshot_t *out_snapshot);
bool blower_metrics_service_capture_zero_offsets(void);
void blower_metrics_service_begin_calibration(void);

float blower_linear_fan_speed_model(float fan_pressure_pa, const void *context);
float blower_linear_air_leakage_model(float fan_speed_units,
                                      float envelope_pressure_pa,
                                      const void *context);

#endif
