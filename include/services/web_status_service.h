#ifndef WEB_STATUS_SERVICE_H
#define WEB_STATUS_SERVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint8_t pwm;
  uint8_t led;
  uint8_t relay;
  uint8_t control_mode;
  uint8_t line_sync;
  float frequency_hz;
  float dp1_pressure_pa;
  float dp1_temperature_c;
  bool dp1_ok;
  float dp2_pressure_pa;
  float dp2_temperature_c;
  bool dp2_ok;
  float fan_wind_speed_ms;
  float fan_wind_speed_kmh;
  float fan_flow_m3h;
  float target_pressure_pa;
  uint32_t sample_sequence;
  uint32_t logs_generation;
  uint8_t test_active;
  uint8_t test_state;
  uint8_t test_mode;
  uint8_t test_direction;
  uint8_t test_point_index;
  uint8_t test_total_points;
  float test_target_pressure_pa;
  float test_measured_pressure_pa;
  float test_measured_flow_m3h;
  uint16_t test_sample_count;
  uint8_t test_report_ready;
  uint32_t test_latest_report_id;
  float test_latest_ach_h1;
} web_status_snapshot_t;

bool web_status_service_collect_snapshot(web_status_snapshot_t *out_snapshot);
bool web_status_service_has_changed(const web_status_snapshot_t *current,
                                    const web_status_snapshot_t *last);
bool web_status_service_format_json(const web_status_snapshot_t *status,
                                    char *payload, size_t payload_size);

#endif
