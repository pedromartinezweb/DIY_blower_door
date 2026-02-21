#include "services/web_status_service.h"

#include "app/app_config.h"
#include "services/blower_control.h"
#include "services/blower_metrics.h"
#include "services/blower_test_service.h"
#include "services/debug_logs.h"
#include "services/http_payload_utils.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define STATUS_FLOAT_TOLERANCE 0.01f
#define FAN_PITOT_NOISE_FLOOR_PA 0.5f

static float web_absf(float value) { return value >= 0.0f ? value : -value; }

static float web_air_density(float altitude_m, float temperature_c) {
  const float alt = altitude_m < 0.0f ? 0.0f : (altitude_m > 6000.0f ? 6000.0f : altitude_m);
  const float p_pa = APP_REFERENCE_PRESSURE_PA * powf(1.0f - 2.25577e-5f * alt, 5.25588f);
  float t_k = temperature_c + 273.15f;
  if (t_k < 233.15f) t_k = 233.15f;
  if (t_k > 353.15f) t_k = 353.15f;
  return p_pa / (APP_AIR_GAS_CONSTANT * t_k);
}

static float web_pitot_wind_speed_ms(float dp_pa, float air_density) {
  const float dp_abs = web_absf(dp_pa);
  if (dp_abs <= 0.0f || air_density <= 0.0f) return 0.0f;
  return sqrtf(2.0f * dp_abs / air_density);
}

bool web_status_service_collect_snapshot(web_status_snapshot_t *out_snapshot) {
  blower_control_snapshot_t control_snapshot = {0};
  blower_metrics_snapshot_t metrics_snapshot = {0};
  blower_test_runtime_status_t test_runtime = {0};
  const bool has_metrics = blower_metrics_service_get_snapshot(&metrics_snapshot);

  if (out_snapshot == NULL) {
    return false;
  }

  blower_control_get_snapshot(&control_snapshot);
  blower_test_service_get_runtime(&test_runtime);

  *out_snapshot = (web_status_snapshot_t){
      .pwm = control_snapshot.output_pwm_percent,
      .led = control_snapshot.mode != BLOWER_CONTROL_MODE_MANUAL_PERCENT ? 1u : 0u,
      .control_mode = (uint8_t)control_snapshot.mode,
      .relay = control_snapshot.relay_enabled ? 1u : 0u,
      .line_sync = control_snapshot.line_sync ? 1u : 0u,
      .frequency_hz = control_snapshot.line_frequency_hz,
      .dp1_pressure_pa = has_metrics ? metrics_snapshot.fan_pressure_pa : 0.0f,
      .dp1_temperature_c = has_metrics ? metrics_snapshot.fan_temperature_c : 0.0f,
      .dp1_ok = has_metrics && metrics_snapshot.fan_sample_valid,
      .dp2_pressure_pa =
          has_metrics ? metrics_snapshot.envelope_pressure_pa : 0.0f,
      .dp2_temperature_c =
          has_metrics ? metrics_snapshot.envelope_temperature_c : 0.0f,
      .dp2_ok = has_metrics && metrics_snapshot.envelope_sample_valid,
      .fan_wind_speed_ms = 0.0f,
      .fan_wind_speed_kmh = 0.0f,
      .fan_flow_m3h = 0.0f,
      .target_pressure_pa = control_snapshot.target_pressure_pa,
      .sample_sequence = has_metrics ? metrics_snapshot.update_sequence : 0u,
      .logs_generation = debug_logs_generation_get(),
      .test_active = test_runtime.active ? 1u : 0u,
      .test_state = (uint8_t)test_runtime.state,
      .test_mode = (uint8_t)test_runtime.requested_mode,
      .test_direction = (uint8_t)test_runtime.current_direction,
      .test_point_index = test_runtime.current_point_index,
      .test_total_points = test_runtime.total_points,
      .test_target_pressure_pa = test_runtime.current_target_pressure_pa,
      .test_measured_pressure_pa = test_runtime.current_measured_pressure_pa,
      .test_measured_flow_m3h = test_runtime.current_measured_flow_m3h,
      .test_sample_count = test_runtime.active_sample_count,
      .test_report_ready = test_runtime.report_ready ? 1u : 0u,
      .test_latest_report_id = test_runtime.latest_report_id,
      .test_latest_ach_h1 = test_runtime.latest_ach_ref_h1,
  };

  if (has_metrics && metrics_snapshot.fan_sample_valid) {
    const float dp_abs = web_absf(metrics_snapshot.fan_pressure_pa);
    if (dp_abs >= FAN_PITOT_NOISE_FLOOR_PA) {
      const float rho = web_air_density(APP_ALTITUDE_M,
                                        metrics_snapshot.fan_temperature_c);
      const float v_ms = web_pitot_wind_speed_ms(metrics_snapshot.fan_pressure_pa,
                                                 rho);
      const float density_factor = rho > 0.0f
          ? sqrtf(APP_SEA_LEVEL_AIR_DENSITY / rho) : 1.0f;

      out_snapshot->fan_wind_speed_ms = v_ms;
      out_snapshot->fan_wind_speed_kmh = v_ms * 3.6f;
      out_snapshot->fan_flow_m3h = APP_FAN_FLOW_COEFFICIENT_C *
            powf(dp_abs, APP_FAN_FLOW_EXPONENT_N) * density_factor;
    }
  }

  return true;
}

bool web_status_service_has_changed(const web_status_snapshot_t *current,
                                    const web_status_snapshot_t *last) {
  if (current == NULL || last == NULL) {
    return true;
  }

  if (current->pwm != last->pwm || current->led != last->led ||
      current->relay != last->relay ||
      current->control_mode != last->control_mode ||
      current->line_sync != last->line_sync ||
      current->dp1_ok != last->dp1_ok || current->dp2_ok != last->dp2_ok ||
      current->test_active != last->test_active ||
      current->test_state != last->test_state ||
      current->test_mode != last->test_mode ||
      current->test_direction != last->test_direction ||
      current->test_point_index != last->test_point_index ||
      current->test_total_points != last->test_total_points ||
      current->test_sample_count != last->test_sample_count ||
      current->test_report_ready != last->test_report_ready ||
      current->test_latest_report_id != last->test_latest_report_id) {
    return true;
  }

  if (web_absf(current->frequency_hz - last->frequency_hz) >
      STATUS_FLOAT_TOLERANCE) {
    return true;
  }
  if (web_absf(current->dp1_pressure_pa - last->dp1_pressure_pa) >
      STATUS_FLOAT_TOLERANCE) {
    return true;
  }
  if (web_absf(current->dp1_temperature_c - last->dp1_temperature_c) >
      STATUS_FLOAT_TOLERANCE) {
    return true;
  }
  if (web_absf(current->dp2_pressure_pa - last->dp2_pressure_pa) >
      STATUS_FLOAT_TOLERANCE) {
    return true;
  }
  if (web_absf(current->dp2_temperature_c - last->dp2_temperature_c) >
      STATUS_FLOAT_TOLERANCE) {
    return true;
  }
  if (web_absf(current->fan_flow_m3h - last->fan_flow_m3h) >
      STATUS_FLOAT_TOLERANCE) {
    return true;
  }
  if (web_absf(current->fan_wind_speed_ms - last->fan_wind_speed_ms) >
      STATUS_FLOAT_TOLERANCE) {
    return true;
  }
  if (web_absf(current->fan_wind_speed_kmh - last->fan_wind_speed_kmh) >
      STATUS_FLOAT_TOLERANCE) {
    return true;
  }
  if (web_absf(current->target_pressure_pa - last->target_pressure_pa) >
      STATUS_FLOAT_TOLERANCE) {
    return true;
  }
  if (web_absf(current->test_target_pressure_pa - last->test_target_pressure_pa) >
      STATUS_FLOAT_TOLERANCE) {
    return true;
  }
  if (web_absf(current->test_measured_pressure_pa -
               last->test_measured_pressure_pa) > STATUS_FLOAT_TOLERANCE) {
    return true;
  }
  if (web_absf(current->test_measured_flow_m3h - last->test_measured_flow_m3h) >
      STATUS_FLOAT_TOLERANCE) {
    return true;
  }
  if (web_absf(current->test_latest_ach_h1 - last->test_latest_ach_h1) >
      STATUS_FLOAT_TOLERANCE) {
    return true;
  }
  if (current->logs_generation != last->logs_generation) {
    return true;
  }

  return false;
}

static int web_status_json_write_common(const web_status_snapshot_t *status,
                                        char *payload, size_t payload_size,
                                        bool logs_enabled,
                                        const char *escaped_logs) {
  if (logs_enabled) {
    return snprintf(
        payload, payload_size,
        "{\"pwm\":%u,\"led\":%u,\"relay\":%u,\"control_mode\":%u,\"line_sync\":%u,"
        "\"input\":%u,\"frequency\":%.1f,\"dp1_pressure\":%.3f,"
        "\"dp1_temperature\":%.3f,\"dp1_ok\":%s,\"dp2_pressure\":%.3f,"
        "\"dp2_temperature\":%.3f,\"dp2_ok\":%s,\"dp_pressure\":%.3f,"
        "\"dp_temperature\":%.3f,\"fan_wind_speed_ms\":%.2f,"
        "\"fan_wind_speed_kmh\":%.2f,\"fan_flow_m3h\":%.3f,"
        "\"target_pressure_pa\":%.2f,"
        "\"test_active\":%u,\"test_state\":%u,\"test_mode\":%u,"
        "\"test_direction\":%u,\"test_point_index\":%u,\"test_total_points\":%u,"
        "\"test_target_pressure\":%.2f,\"test_measured_pressure\":%.2f,"
        "\"test_measured_flow_m3h\":%.2f,\"test_sample_count\":%u,"
        "\"test_report_ready\":%u,\"test_latest_report_id\":%lu,"
        "\"test_latest_ach_h1\":%.3f,"
        "\"logs_enabled\":true,\"logs\":\"%s\"}",
        status->pwm, status->led, status->relay, status->control_mode, status->line_sync,
        status->line_sync, status->frequency_hz, status->dp1_pressure_pa,
        status->dp1_temperature_c, status->dp1_ok ? "true" : "false",
        status->dp2_pressure_pa, status->dp2_temperature_c,
        status->dp2_ok ? "true" : "false", status->dp1_pressure_pa,
        status->dp1_temperature_c, status->fan_wind_speed_ms,
        status->fan_wind_speed_kmh, status->fan_flow_m3h,
        status->target_pressure_pa, status->test_active, status->test_state,
        status->test_mode, status->test_direction, status->test_point_index,
        status->test_total_points, status->test_target_pressure_pa,
        status->test_measured_pressure_pa, status->test_measured_flow_m3h,
        status->test_sample_count, status->test_report_ready,
        (unsigned long)status->test_latest_report_id,
        status->test_latest_ach_h1, escaped_logs != NULL ? escaped_logs : "");
  }

  return snprintf(
      payload, payload_size,
      "{\"pwm\":%u,\"led\":%u,\"relay\":%u,\"control_mode\":%u,\"line_sync\":%u,\"input\":%u,"
      "\"frequency\":%.1f,\"dp1_pressure\":%.3f,\"dp1_temperature\":%.3f,"
      "\"dp1_ok\":%s,\"dp2_pressure\":%.3f,\"dp2_temperature\":%.3f,"
      "\"dp2_ok\":%s,\"dp_pressure\":%.3f,\"dp_temperature\":%.3f,"
      "\"fan_wind_speed_ms\":%.2f,\"fan_wind_speed_kmh\":%.2f,"
      "\"fan_flow_m3h\":%.3f,\"target_pressure_pa\":%.2f,"
      "\"test_active\":%u,\"test_state\":%u,\"test_mode\":%u,"
      "\"test_direction\":%u,\"test_point_index\":%u,\"test_total_points\":%u,"
      "\"test_target_pressure\":%.2f,\"test_measured_pressure\":%.2f,"
      "\"test_measured_flow_m3h\":%.2f,\"test_sample_count\":%u,"
      "\"test_report_ready\":%u,\"test_latest_report_id\":%lu,"
      "\"test_latest_ach_h1\":%.3f,"
      "\"logs_enabled\":false}",
      status->pwm, status->led, status->relay, status->control_mode, status->line_sync,
      status->line_sync, status->frequency_hz, status->dp1_pressure_pa,
      status->dp1_temperature_c, status->dp1_ok ? "true" : "false",
      status->dp2_pressure_pa, status->dp2_temperature_c,
      status->dp2_ok ? "true" : "false", status->dp1_pressure_pa,
      status->dp1_temperature_c, status->fan_wind_speed_ms,
      status->fan_wind_speed_kmh, status->fan_flow_m3h,
      status->target_pressure_pa, status->test_active, status->test_state,
      status->test_mode, status->test_direction, status->test_point_index,
      status->test_total_points, status->test_target_pressure_pa,
      status->test_measured_pressure_pa, status->test_measured_flow_m3h,
      status->test_sample_count, status->test_report_ready,
      (unsigned long)status->test_latest_report_id,
      status->test_latest_ach_h1);
}

bool web_status_service_format_json(const web_status_snapshot_t *status,
                                    char *payload, size_t payload_size) {
  bool logs_enabled = debug_logs_enabled_get();
  int written = 0;

  if (status == NULL || payload == NULL || payload_size == 0u) {
    return false;
  }

#if !APP_ENABLE_DEBUG_HTTP_ROUTES
  logs_enabled = false;
#endif

  if (!logs_enabled) {
    written = web_status_json_write_common(status, payload, payload_size, false,
                                           NULL);
    return written > 0 && (size_t)written < payload_size;
  }

  {
    char logs_tail[DEBUG_LOG_TAIL_CHARS + 1u];
    char escaped_logs[(DEBUG_LOG_TAIL_CHARS * 2u) + 1u];
    size_t escaped_length = 0u;

    debug_logs_copy_tail(logs_tail, sizeof(logs_tail));
    if (!json_escape_string(logs_tail, escaped_logs, sizeof(escaped_logs))) {
      return false;
    }

    escaped_length = strlen(escaped_logs);
    while (1) {
      escaped_logs[escaped_length] = '\0';
      written = web_status_json_write_common(status, payload, payload_size, true,
                                             escaped_logs);
      if (written > 0 && (size_t)written < payload_size) {
        return true;
      }

      if (escaped_length == 0u) {
        break;
      }

      escaped_length /= 2u;
    }
  }

  written = web_status_json_write_common(status, payload, payload_size, true, "");
  return written > 0 && (size_t)written < payload_size;
}
