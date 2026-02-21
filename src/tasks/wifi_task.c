#include "tasks/task_entries.h"

#include "app/app_config.h"
#include "FreeRTOS.h"
#include "hardware/sync.h"
#include "lwip/api.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "pico/cyw43_arch.h"
#include "services/blower_control.h"
#include "services/blower_metrics.h"
#include "services/ota_update_service.h"
#include "task.h"
#include "web/web_assets.h"
#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#ifndef WIFI_SSID
#define WIFI_SSID "WIFI_SSID"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "WIFI_PASSWORD"
#endif

#define WIFI_CONNECT_TIMEOUT_MS 30000u
#define WIFI_RETRY_DELAY_MS 1000u
#define HTTP_SERVER_PORT 80u

#define HTTP_REQUEST_LINE_BUFFER_SIZE 256u
#define HTTP_REQUEST_BUFFER_SIZE 6144u
#define HTTP_MAX_BODY_SIZE 4096u
#define HTTP_RESPONSE_PAYLOAD_BUFFER_SIZE 1024u
#define HTTP_RESPONSE_CHUNK_SIZE 1024u

#define SSE_LOOP_INTERVAL_MS 250u
#define SSE_FORCE_PUBLISH_INTERVAL_MS 1000u
#define STATUS_FLOAT_TOLERANCE 0.01f

#define DEBUG_LOG_BUFFER_SIZE 1024u
#define DEBUG_LOG_TAIL_CHARS 192u
#define OTA_MAX_DECODED_CHUNK_BYTES 3072u

typedef enum {
  HTTP_METHOD_UNKNOWN = 0,
  HTTP_METHOD_GET,
  HTTP_METHOD_HEAD,
  HTTP_METHOD_POST,
} http_method_t;

typedef struct {
  http_method_t method;
  char path[96];
  char body[HTTP_MAX_BODY_SIZE + 1u];
  size_t body_length;
} http_request_t;

typedef struct {
  uint8_t pwm;
  uint8_t led;
  uint8_t relay;
  uint8_t line_sync;
  float frequency_hz;
  float dp1_pressure_pa;
  float dp1_temperature_c;
  bool dp1_ok;
  float dp2_pressure_pa;
  float dp2_temperature_c;
  bool dp2_ok;
  float fan_flow_m3h;
  float target_pressure_pa;
  uint32_t sample_sequence;
  uint32_t logs_generation;
  uint8_t cal_state;
  uint8_t cal_pct;
  float cal_fan_offset;
  float cal_env_offset;
} web_status_snapshot_t;

typedef struct {
  struct netconn *connection;
  web_status_snapshot_t last_status;
  bool has_last_status;
  uint32_t last_emit_ms;
} sse_stream_context_t;

static volatile bool g_sse_active = false;
static volatile bool g_sse_stop_requested = false;
#if APP_ENABLE_DEBUG_HTTP_ROUTES
static volatile bool g_debug_logs_enabled = false;
static volatile uint32_t g_debug_logs_generation = 0u;
static char g_debug_log_buffer[DEBUG_LOG_BUFFER_SIZE];
static size_t g_debug_log_length = 0u;
#endif
static uint8_t g_ota_decoded_chunk_buffer[OTA_MAX_DECODED_CHUNK_BYTES];

static float web_absf(float value) { return value >= 0.0f ? value : -value; }

#if APP_ENABLE_DEBUG_HTTP_ROUTES
static void debug_logs_clear(void) {
  const uint32_t irq_state = save_and_disable_interrupts();
  g_debug_log_length = 0u;
  g_debug_log_buffer[0] = '\0';
  g_debug_logs_generation += 1u;
  restore_interrupts(irq_state);
}

static void debug_logs_append(const char *line) {
  size_t line_length = 0u;
  size_t overflow = 0u;
  const uint32_t irq_state = save_and_disable_interrupts();

  if (line == NULL || !g_debug_logs_enabled) {
    restore_interrupts(irq_state);
    return;
  }

  line_length = strlen(line);
  if (line_length + 1u >= DEBUG_LOG_BUFFER_SIZE) {
    line += line_length - (DEBUG_LOG_BUFFER_SIZE - 2u);
    line_length = DEBUG_LOG_BUFFER_SIZE - 2u;
  }

  if (g_debug_log_length + line_length + 1u >= DEBUG_LOG_BUFFER_SIZE) {
    overflow = g_debug_log_length + line_length + 1u - DEBUG_LOG_BUFFER_SIZE;
    if (overflow > g_debug_log_length) {
      overflow = g_debug_log_length;
    }
    if (overflow > 0u) {
      memmove(g_debug_log_buffer, g_debug_log_buffer + overflow,
              g_debug_log_length - overflow);
      g_debug_log_length -= overflow;
    }
  }

  memcpy(g_debug_log_buffer + g_debug_log_length, line, line_length);
  g_debug_log_length += line_length;
  g_debug_log_buffer[g_debug_log_length++] = '\n';
  g_debug_log_buffer[g_debug_log_length] = '\0';
  g_debug_logs_generation += 1u;

  restore_interrupts(irq_state);
}

static void debug_logs_copy(char *out_buffer, size_t out_buffer_size) {
  const uint32_t irq_state = save_and_disable_interrupts();
  size_t copy_length = 0u;

  if (out_buffer == NULL || out_buffer_size == 0u) {
    restore_interrupts(irq_state);
    return;
  }

  copy_length = g_debug_log_length;
  if (copy_length >= out_buffer_size) {
    copy_length = out_buffer_size - 1u;
  }

  memcpy(out_buffer, g_debug_log_buffer, copy_length);
  out_buffer[copy_length] = '\0';

  restore_interrupts(irq_state);
}

static bool debug_logs_enabled_get(void) {
  const uint32_t irq_state = save_and_disable_interrupts();
  const bool enabled = g_debug_logs_enabled;
  restore_interrupts(irq_state);
  return enabled;
}

static uint32_t debug_logs_generation_get(void) {
  const uint32_t irq_state = save_and_disable_interrupts();
  const uint32_t generation = g_debug_logs_generation;
  restore_interrupts(irq_state);
  return generation;
}

static void debug_logs_enabled_set(bool enabled) {
  const uint32_t irq_state = save_and_disable_interrupts();
  g_debug_logs_enabled = enabled;
  restore_interrupts(irq_state);
}

static void debug_logs_copy_tail(char *out_buffer, size_t out_buffer_size) {
  const uint32_t irq_state = save_and_disable_interrupts();
  size_t copy_length = 0u;
  size_t start_index = 0u;

  if (out_buffer == NULL || out_buffer_size == 0u) {
    restore_interrupts(irq_state);
    return;
  }

  if (g_debug_log_length > 0u) {
    const size_t max_copy = out_buffer_size - 1u;
    if (g_debug_log_length > max_copy) {
      start_index = g_debug_log_length - max_copy;
      copy_length = max_copy;
    } else {
      copy_length = g_debug_log_length;
    }
    memcpy(out_buffer, g_debug_log_buffer + start_index, copy_length);
  }

  out_buffer[copy_length] = '\0';
  restore_interrupts(irq_state);
}
#else
static void debug_logs_clear(void) {}
static void debug_logs_append(const char *line) { (void)line; }
static void debug_logs_copy(char *out_buffer, size_t out_buffer_size) {
  if (out_buffer != NULL && out_buffer_size > 0u) {
    out_buffer[0] = '\0';
  }
}
static bool debug_logs_enabled_get(void) { return false; }
static uint32_t debug_logs_generation_get(void) { return 0u; }
static void debug_logs_enabled_set(bool enabled) { (void)enabled; }
static void debug_logs_copy_tail(char *out_buffer, size_t out_buffer_size) {
  if (out_buffer != NULL && out_buffer_size > 0u) {
    out_buffer[0] = '\0';
  }
}
#endif

static bool json_escape_string(const char *input, char *output,
                               size_t output_size) {
  size_t write_index = 0u;
  size_t read_index = 0u;

  if (input == NULL || output == NULL || output_size == 0u) {
    return false;
  }

  while (input[read_index] != '\0') {
    char ch = input[read_index++];
    const char *replacement = NULL;
    char escaped[7];
    size_t replacement_length = 0u;

    switch (ch) {
    case '\\':
      replacement = "\\\\";
      break;
    case '"':
      replacement = "\\\"";
      break;
    case '\n':
      replacement = "\\n";
      break;
    case '\r':
      replacement = "\\r";
      break;
    case '\t':
      replacement = "\\t";
      break;
    default:
      if ((unsigned char)ch < 0x20u) {
        const int written = snprintf(escaped, sizeof(escaped), "\\u%04x",
                                     (unsigned char)ch);
        if (written <= 0 || (size_t)written >= sizeof(escaped)) {
          return false;
        }
        replacement = escaped;
      }
      break;
    }

    if (replacement != NULL) {
      replacement_length = strlen(replacement);
      if (write_index + replacement_length >= output_size) {
        return false;
      }
      memcpy(output + write_index, replacement, replacement_length);
      write_index += replacement_length;
    } else {
      if (write_index + 1u >= output_size) {
        return false;
      }
      output[write_index++] = ch;
    }
  }

  output[write_index] = '\0';
  return true;
}

static void http_send_response(struct netconn *connection, const char *status_line,
                               const char *content_type, const uint8_t *body,
                               size_t body_length) {
  char header[192];
  size_t offset = 0u;
  const int header_length = snprintf(
      header, sizeof(header),
      "HTTP/1.1 %s\r\n"
      "Content-Type: %s\r\n"
      "Content-Length: %lu\r\n"
      "Connection: close\r\n"
      "\r\n",
      status_line, content_type, (unsigned long)body_length);

  if (header_length <= 0 || (size_t)header_length >= sizeof(header)) {
    return;
  }

  if (netconn_write(connection, header, (size_t)header_length, NETCONN_COPY) !=
      ERR_OK) {
    return;
  }

  while (body != NULL && offset < body_length) {
    const size_t remaining = body_length - offset;
    const size_t chunk_size =
        remaining > HTTP_RESPONSE_CHUNK_SIZE ? HTTP_RESPONSE_CHUNK_SIZE
                                             : remaining;
    if (netconn_write(connection, body + offset, chunk_size, NETCONN_COPY) !=
        ERR_OK) {
      return;
    }
    offset += chunk_size;
  }
}

static void http_send_text_response(struct netconn *connection,
                                    const char *status_line,
                                    const char *content_type,
                                    const char *body) {
  http_send_response(connection, status_line, content_type,
                     (const uint8_t *)body, strlen(body));
}

static void http_send_headers_only(struct netconn *connection,
                                   const char *status_line,
                                   const char *content_type,
                                   size_t content_length) {
  char header[192];
  const int header_length = snprintf(
      header, sizeof(header),
      "HTTP/1.1 %s\r\n"
      "Content-Type: %s\r\n"
      "Content-Length: %lu\r\n"
      "Connection: close\r\n"
      "\r\n",
      status_line, content_type, (unsigned long)content_length);

  if (header_length <= 0 || (size_t)header_length >= sizeof(header)) {
    return;
  }

  netconn_write(connection, header, (size_t)header_length, NETCONN_COPY);
}

static bool http_parse_request_path_and_method(const char *request_data,
                                               size_t request_length,
                                               http_method_t *out_method,
                                               char *out_path,
                                               size_t out_path_size) {
  char request_line[HTTP_REQUEST_LINE_BUFFER_SIZE];
  const char *method_prefix = NULL;
  char *path_begin = NULL;
  char *path_end = NULL;
  char *absolute_path_start = NULL;
  char *query_separator = NULL;
  size_t copy_length = request_length;
  size_t path_length = 0u;

  if (request_data == NULL || out_method == NULL || out_path == NULL ||
      out_path_size == 0u || request_length < 5u) {
    return false;
  }

  if (copy_length >= sizeof(request_line)) {
    copy_length = sizeof(request_line) - 1u;
  }

  memcpy(request_line, request_data, copy_length);
  request_line[copy_length] = '\0';

  if (strncmp(request_line, "GET ", 4) == 0) {
    *out_method = HTTP_METHOD_GET;
    method_prefix = "GET ";
  } else if (strncmp(request_line, "HEAD ", 5) == 0) {
    *out_method = HTTP_METHOD_HEAD;
    method_prefix = "HEAD ";
  } else if (strncmp(request_line, "POST ", 5) == 0) {
    *out_method = HTTP_METHOD_POST;
    method_prefix = "POST ";
  } else {
    *out_method = HTTP_METHOD_UNKNOWN;
    return false;
  }

  path_begin = request_line + strlen(method_prefix);
  path_end = strchr(path_begin, ' ');
  if (path_end == NULL || path_end == path_begin) {
    return false;
  }

  path_length = (size_t)(path_end - path_begin);
  if (path_length >= out_path_size) {
    path_length = out_path_size - 1u;
  }

  memcpy(out_path, path_begin, path_length);
  out_path[path_length] = '\0';

  if (strncmp(out_path, "http://", 7) == 0 ||
      strncmp(out_path, "https://", 8) == 0) {
    absolute_path_start = strstr(out_path, "://");
    if (absolute_path_start != NULL) {
      absolute_path_start += 3;
      absolute_path_start = strchr(absolute_path_start, '/');
      if (absolute_path_start != NULL) {
        memmove(out_path, absolute_path_start, strlen(absolute_path_start) + 1u);
      } else {
        strcpy(out_path, "/");
      }
    }
  }

  query_separator = strchr(out_path, '?');
  if (query_separator != NULL) {
    *query_separator = '\0';
  }

  if (out_path[0] == '\0') {
    strcpy(out_path, "/");
  }

  return true;
}

static size_t http_extract_content_length(const char *buffer, size_t header_size) {
  const char *cursor = buffer;
  const char *end = buffer + header_size;

  while (cursor < end) {
    const char *line_end = strstr(cursor, "\r\n");
    size_t line_length = 0u;

    if (line_end == NULL || line_end > end) {
      break;
    }

    line_length = (size_t)(line_end - cursor);
    if (line_length == 0u) {
      break;
    }

    if (line_length > 15u && strncasecmp(cursor, "Content-Length:", 15u) == 0) {
      const char *value_start = cursor + 15u;
      while (value_start < line_end && isspace((unsigned char)*value_start)) {
        value_start++;
      }
      return (size_t)strtoul(value_start, NULL, 10);
    }

    cursor = line_end + 2;
  }

  return 0u;
}

static bool http_receive_request(struct netconn *connection, char *request_buffer,
                                 size_t request_buffer_size, size_t *out_total_size,
                                 size_t *out_header_size,
                                 size_t *out_content_length) {
  size_t total_size = 0u;
  size_t header_size = 0u;
  size_t content_length = 0u;
  bool headers_ready = false;
  bool request_complete = false;

  if (connection == NULL || request_buffer == NULL || out_total_size == NULL ||
      out_header_size == NULL || out_content_length == NULL ||
      request_buffer_size < 8u) {
    return false;
  }

  request_buffer[0] = '\0';
  *out_total_size = 0u;
  *out_header_size = 0u;
  *out_content_length = 0u;

  while (!request_complete && total_size < (request_buffer_size - 1u)) {
    struct netbuf *input_buffer = NULL;
    err_t receive_status = netconn_recv(connection, &input_buffer);

    if (receive_status != ERR_OK || input_buffer == NULL) {
      break;
    }

    do {
      char *chunk_data = NULL;
      u16_t chunk_length = 0u;
      size_t writable_length = 0u;

      netbuf_data(input_buffer, (void **)&chunk_data, &chunk_length);
      if (chunk_data == NULL || chunk_length == 0u) {
        continue;
      }

      writable_length = request_buffer_size - 1u - total_size;
      if (writable_length == 0u) {
        break;
      }

      if ((size_t)chunk_length > writable_length) {
        chunk_length = (u16_t)writable_length;
      }

      memcpy(request_buffer + total_size, chunk_data, (size_t)chunk_length);
      total_size += (size_t)chunk_length;
      request_buffer[total_size] = '\0';

      if (!headers_ready) {
        char *header_end = strstr(request_buffer, "\r\n\r\n");
        if (header_end != NULL) {
          headers_ready = true;
          header_size = (size_t)(header_end - request_buffer) + 4u;
          content_length = http_extract_content_length(request_buffer, header_size);

          if (header_size + content_length > request_buffer_size - 1u) {
            netbuf_delete(input_buffer);
            return false;
          }
        }
      }

      if (headers_ready && total_size >= header_size + content_length) {
        request_complete = true;
        break;
      }
    } while (netbuf_next(input_buffer) >= 0);

    netbuf_delete(input_buffer);
  }

  if (!headers_ready || total_size < header_size + content_length) {
    return false;
  }

  *out_total_size = header_size + content_length;
  *out_header_size = header_size;
  *out_content_length = content_length;
  return true;
}

static bool json_extract_int_field(const char *json_body, const char *field_name,
                                   int *out_value) {
  char token[32];
  const char *field = NULL;
  const char *colon = NULL;
  char *end_ptr = NULL;
  long parsed_value = 0;

  if (json_body == NULL || field_name == NULL || out_value == NULL) {
    return false;
  }

  if (snprintf(token, sizeof(token), "\"%s\"", field_name) <= 0) {
    return false;
  }

  field = strstr(json_body, token);
  if (field == NULL) {
    return false;
  }

  colon = strchr(field, ':');
  if (colon == NULL) {
    return false;
  }

  colon++;
  while (*colon != '\0' && isspace((unsigned char)*colon)) {
    colon++;
  }

  parsed_value = strtol(colon, &end_ptr, 10);
  if (end_ptr == colon) {
    return false;
  }

  *out_value = (int)parsed_value;
  return true;
}

static bool json_extract_bool_field(const char *json_body, const char *field_name,
                                    bool *out_value) {
  char token[32];
  const char *field = NULL;
  const char *colon = NULL;

  if (json_body == NULL || field_name == NULL || out_value == NULL) {
    return false;
  }

  if (snprintf(token, sizeof(token), "\"%s\"", field_name) <= 0) {
    return false;
  }

  field = strstr(json_body, token);
  if (field == NULL) {
    return false;
  }

  colon = strchr(field, ':');
  if (colon == NULL) {
    return false;
  }

  colon++;
  while (*colon != '\0' && isspace((unsigned char)*colon)) {
    colon++;
  }

  if (strncmp(colon, "true", 4u) == 0) {
    *out_value = true;
    return true;
  }

  if (strncmp(colon, "false", 5u) == 0) {
    *out_value = false;
    return true;
  }

  if (*colon == '1') {
    *out_value = true;
    return true;
  }

  if (*colon == '0') {
    *out_value = false;
    return true;
  }

  return false;
}

static bool json_extract_uint32_field(const char *json_body,
                                      const char *field_name,
                                      uint32_t *out_value) {
  char token[32];
  const char *field = NULL;
  const char *colon = NULL;
  char *end_ptr = NULL;
  unsigned long parsed_value = 0u;

  if (json_body == NULL || field_name == NULL || out_value == NULL) {
    return false;
  }

  if (snprintf(token, sizeof(token), "\"%s\"", field_name) <= 0) {
    return false;
  }

  field = strstr(json_body, token);
  if (field == NULL) {
    return false;
  }

  colon = strchr(field, ':');
  if (colon == NULL) {
    return false;
  }

  colon++;
  while (*colon != '\0' && isspace((unsigned char)*colon)) {
    colon++;
  }

  if (*colon == '-') {
    return false;
  }

  parsed_value = strtoul(colon, &end_ptr, 10);
  if (end_ptr == colon || parsed_value > 0xfffffffful) {
    return false;
  }

  *out_value = (uint32_t)parsed_value;
  return true;
}

static bool json_extract_string_field(const char *json_body,
                                      const char *field_name, char *out_value,
                                      size_t out_value_size) {
  char token[32];
  const char *field = NULL;
  const char *colon = NULL;
  const char *cursor = NULL;
  size_t write_index = 0u;

  if (json_body == NULL || field_name == NULL || out_value == NULL ||
      out_value_size == 0u) {
    return false;
  }

  if (snprintf(token, sizeof(token), "\"%s\"", field_name) <= 0) {
    return false;
  }

  field = strstr(json_body, token);
  if (field == NULL) {
    return false;
  }

  colon = strchr(field, ':');
  if (colon == NULL) {
    return false;
  }

  cursor = colon + 1;
  while (*cursor != '\0' && isspace((unsigned char)*cursor)) {
    cursor++;
  }

  if (*cursor != '"') {
    return false;
  }
  cursor++;

  while (*cursor != '\0' && *cursor != '"') {
    char ch = *cursor++;
    if (ch == '\\') {
      if (*cursor == '\0') {
        return false;
      }
      ch = *cursor++;
    }

    if (write_index + 1u >= out_value_size) {
      return false;
    }
    out_value[write_index++] = ch;
  }

  if (*cursor != '"') {
    return false;
  }

  out_value[write_index] = '\0';
  return true;
}

static int base64_decode_char(char value) {
  if (value >= 'A' && value <= 'Z') {
    return value - 'A';
  }
  if (value >= 'a' && value <= 'z') {
    return (value - 'a') + 26;
  }
  if (value >= '0' && value <= '9') {
    return (value - '0') + 52;
  }
  if (value == '+') {
    return 62;
  }
  if (value == '/') {
    return 63;
  }
  if (value == '=') {
    return -2;
  }
  return -1;
}

static bool base64_decode_payload(const char *input, uint8_t *output,
                                  size_t output_capacity,
                                  size_t *out_output_length) {
  int values[4];
  size_t output_length = 0u;
  size_t values_count = 0u;
  bool found_padding = false;
  const char *cursor = input;

  if (input == NULL || output == NULL || out_output_length == NULL) {
    return false;
  }

  while (*cursor != '\0') {
    const int decoded = base64_decode_char(*cursor++);
    if (decoded < -1) {
      found_padding = true;
    }
    if (decoded == -1) {
      if (!isspace((unsigned char)cursor[-1])) {
        return false;
      }
      continue;
    }
    if (found_padding && decoded >= 0) {
      return false;
    }

    values[values_count++] = decoded;
    if (values_count < 4u) {
      continue;
    }

    if (values[0] < 0 || values[1] < 0) {
      return false;
    }
    if (output_length + 1u > output_capacity) {
      return false;
    }
    output[output_length++] =
        (uint8_t)((values[0] << 2) | ((values[1] & 0x30) >> 4));

    if (values[2] == -2) {
      if (values[3] != -2) {
        return false;
      }
    } else {
      if (values[2] < 0) {
        return false;
      }
      if (output_length + 1u > output_capacity) {
        return false;
      }
      output[output_length++] =
          (uint8_t)(((values[1] & 0x0f) << 4) | ((values[2] & 0x3c) >> 2));

      if (values[3] != -2) {
        if (values[3] < 0) {
          return false;
        }
        if (output_length + 1u > output_capacity) {
          return false;
        }
        output[output_length++] =
            (uint8_t)(((values[2] & 0x03) << 6) | values[3]);
      }
    }

    values_count = 0u;
  }

  if (values_count != 0u) {
    return false;
  }

  *out_output_length = output_length;
  return true;
}

static bool web_collect_status_snapshot(web_status_snapshot_t *out_snapshot) {
  blower_control_snapshot_t control_snapshot = {0};
  blower_metrics_snapshot_t metrics_snapshot = {0};
  const bool has_metrics = blower_metrics_service_get_snapshot(&metrics_snapshot);

  if (out_snapshot == NULL) {
    return false;
  }

  blower_control_get_snapshot(&control_snapshot);

  *out_snapshot = (web_status_snapshot_t){
      .pwm = control_snapshot.output_pwm_percent,
      .led = control_snapshot.auto_hold_enabled ? 1u : 0u,
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
      .fan_flow_m3h =
          has_metrics && metrics_snapshot.fan_sample_valid
              ? APP_FAN_FLOW_COEFFICIENT_C *
                    powf(web_absf(metrics_snapshot.fan_pressure_pa),
                         APP_FAN_FLOW_EXPONENT_N)
              : 0.0f,
      .target_pressure_pa = control_snapshot.target_pressure_pa,
      .sample_sequence = has_metrics ? metrics_snapshot.update_sequence : 0u,
      .logs_generation = debug_logs_generation_get(),
      .cal_state = has_metrics ? (uint8_t)metrics_snapshot.calibration_state : 0u,
      .cal_pct = has_metrics ? metrics_snapshot.calibration_progress_pct : 0u,
      .cal_fan_offset = has_metrics ? metrics_snapshot.calibration_fan_offset : 0.0f,
      .cal_env_offset = has_metrics ? metrics_snapshot.calibration_envelope_offset : 0.0f,
  };

  return true;
}

static bool web_status_changed(const web_status_snapshot_t *current,
                               const web_status_snapshot_t *last) {
  if (current == NULL || last == NULL) {
    return true;
  }

  if (current->pwm != last->pwm || current->led != last->led ||
      current->relay != last->relay || current->line_sync != last->line_sync ||
      current->dp1_ok != last->dp1_ok || current->dp2_ok != last->dp2_ok ||
      current->cal_state != last->cal_state ||
      current->cal_pct != last->cal_pct) {
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
  if (web_absf(current->target_pressure_pa - last->target_pressure_pa) >
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
        "{\"fw\":\"" APP_FIRMWARE_VERSION "\","
        "\"pwm\":%u,\"led\":%u,\"relay\":%u,\"line_sync\":%u,"
        "\"input\":%u,\"frequency\":%.1f,\"dp1_pressure\":%.3f,"
        "\"dp1_temperature\":%.3f,\"dp1_ok\":%s,\"dp2_pressure\":%.3f,"
        "\"dp2_temperature\":%.3f,\"dp2_ok\":%s,\"dp_pressure\":%.3f,"
        "\"dp_temperature\":%.3f,\"fan_flow_m3h\":%.3f,"
        "\"target_pressure_pa\":%.2f,\"sample_sequence\":%lu,"
        "\"cal\":%u,\"cal_pct\":%u,"
        "\"cal_fan\":%.3f,\"cal_env\":%.3f,"
        "\"logs_enabled\":true,\"logs\":\"%s\"}",
        status->pwm, status->led, status->relay, status->line_sync,
        status->line_sync, status->frequency_hz, status->dp1_pressure_pa,
        status->dp1_temperature_c, status->dp1_ok ? "true" : "false",
        status->dp2_pressure_pa, status->dp2_temperature_c,
        status->dp2_ok ? "true" : "false", status->dp1_pressure_pa,
        status->dp1_temperature_c, status->fan_flow_m3h,
        status->target_pressure_pa, (unsigned long)status->sample_sequence,
        (unsigned)status->cal_state, (unsigned)status->cal_pct,
        status->cal_fan_offset, status->cal_env_offset,
        escaped_logs != NULL ? escaped_logs : "");
  }

  return snprintf(
      payload, payload_size,
      "{\"fw\":\"" APP_FIRMWARE_VERSION "\","
      "\"pwm\":%u,\"led\":%u,\"relay\":%u,\"line_sync\":%u,\"input\":%u,"
      "\"frequency\":%.1f,\"dp1_pressure\":%.3f,\"dp1_temperature\":%.3f,"
      "\"dp1_ok\":%s,\"dp2_pressure\":%.3f,\"dp2_temperature\":%.3f,"
      "\"dp2_ok\":%s,\"dp_pressure\":%.3f,\"dp_temperature\":%.3f,"
      "\"fan_flow_m3h\":%.3f,\"target_pressure_pa\":%.2f,"
      "\"sample_sequence\":%lu,"
      "\"cal\":%u,\"cal_pct\":%u,"
      "\"cal_fan\":%.3f,\"cal_env\":%.3f,"
      "\"logs_enabled\":false}",
      status->pwm, status->led, status->relay, status->line_sync,
      status->line_sync, status->frequency_hz, status->dp1_pressure_pa,
      status->dp1_temperature_c, status->dp1_ok ? "true" : "false",
      status->dp2_pressure_pa, status->dp2_temperature_c,
      status->dp2_ok ? "true" : "false", status->dp1_pressure_pa,
      status->dp1_temperature_c, status->fan_flow_m3h,
      status->target_pressure_pa, (unsigned long)status->sample_sequence,
      (unsigned)status->cal_state, (unsigned)status->cal_pct,
      status->cal_fan_offset, status->cal_env_offset);
}

static bool web_format_status_json(const web_status_snapshot_t *status,
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

static bool sse_write_event(struct netconn *connection, const char *json_payload) {
  static const char k_prefix[] = "data:";
  static const char k_suffix[] = "\n\n";

  if (connection == NULL || json_payload == NULL) {
    return false;
  }

  if (netconn_write(connection, k_prefix, sizeof(k_prefix) - 1u, NETCONN_COPY) !=
      ERR_OK) {
    return false;
  }
  if (netconn_write(connection, json_payload, strlen(json_payload), NETCONN_COPY) !=
      ERR_OK) {
    return false;
  }
  if (netconn_write(connection, k_suffix, sizeof(k_suffix) - 1u, NETCONN_COPY) !=
      ERR_OK) {
    return false;
  }

  return true;
}

static void http_send_sse_headers(struct netconn *connection) {
  static const char k_sse_headers[] =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/event-stream\r\n"
      "Cache-Control: no-store, no-cache, must-revalidate\r\n"
      "Connection: keep-alive\r\n"
      "X-Accel-Buffering: no\r\n"
      "\r\n";
  netconn_write(connection, k_sse_headers, sizeof(k_sse_headers) - 1u,
                NETCONN_COPY);
}

static void sse_stream_task(void *params) {
  sse_stream_context_t *context = (sse_stream_context_t *)params;
  struct netconn *connection = NULL;
  char json_payload[HTTP_RESPONSE_PAYLOAD_BUFFER_SIZE];
  const char *close_reason = "stop_requested";
  uint32_t sent_events = 0u;

  if (context == NULL) {
    printf("[SSE] close reason=context_null\n");
    g_sse_active = false;
    vTaskDelete(NULL);
    return;
  }

  connection = context->connection;
  if (connection == NULL) {
    printf("[SSE] close reason=connection_null\n");
    g_sse_active = false;
    vPortFree(context);
    vTaskDelete(NULL);
    return;
  }

  http_send_sse_headers(connection);
  printf("[SSE] opened\n");
  context->last_emit_ms = to_ms_since_boot(get_absolute_time());

  while (1) {
    if (g_sse_stop_requested) {
      close_reason = "stop_requested";
      break;
    }

    web_status_snapshot_t status_snapshot = {0};
    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    const bool has_status = web_collect_status_snapshot(&status_snapshot);
    const bool should_push =
        has_status &&
        (!context->has_last_status ||
         web_status_changed(&status_snapshot, &context->last_status) ||
         (now_ms - context->last_emit_ms) >= SSE_FORCE_PUBLISH_INTERVAL_MS);

    if (should_push) {
      const bool json_ok = web_format_status_json(&status_snapshot, json_payload,
                                                  sizeof(json_payload));
      if (!json_ok) {
        static const char k_fallback_payload[] =
            "{\"logs_enabled\":false,\"error\":\"payload\"}";
        debug_logs_append("SSE payload fallback");
        if (!sse_write_event(connection, k_fallback_payload)) {
          debug_logs_append("SSE write fail fallback");
          close_reason = "write_fail_fallback";
          break;
        }
        sent_events += 1u;
        context->last_emit_ms = now_ms;
        vTaskDelay(pdMS_TO_TICKS(SSE_LOOP_INTERVAL_MS));
        continue;
      }

      if (!sse_write_event(connection, json_payload)) {
        debug_logs_append("SSE write fail data");
        close_reason = "write_fail_data";
        break;
      }

      sent_events += 1u;
      if ((sent_events % 10u) == 0u) {
        printf("[SSE] sent=%lu seq=%lu fan_ok=%u env_ok=%u\n",
               (unsigned long)sent_events,
               (unsigned long)status_snapshot.sample_sequence,
               status_snapshot.dp1_ok ? 1u : 0u,
               status_snapshot.dp2_ok ? 1u : 0u);
      }

      context->last_status = status_snapshot;
      context->has_last_status = true;
      context->last_emit_ms = now_ms;
    }

    vTaskDelay(pdMS_TO_TICKS(SSE_LOOP_INTERVAL_MS));
  }

  netconn_close(connection);
  netconn_delete(connection);
  printf("[SSE] closed reason=%s sent=%lu\n", close_reason,
         (unsigned long)sent_events);
  debug_logs_append("SSE closed");
  g_sse_stop_requested = false;
  g_sse_active = false;
  vPortFree(context);
  vTaskDelete(NULL);
}

static bool http_start_sse_stream(struct netconn *connection) {
  sse_stream_context_t *context = NULL;

  if (g_sse_active) {
    const uint32_t wait_start_ms = to_ms_since_boot(get_absolute_time());
    const uint32_t handover_timeout_ms = 1200u;

    printf("[SSE] handover requested\n");
    g_sse_stop_requested = true;
    while (g_sse_active &&
           (to_ms_since_boot(get_absolute_time()) - wait_start_ms) <
               handover_timeout_ms) {
      vTaskDelay(pdMS_TO_TICKS(20u));
    }

    if (g_sse_active) {
      printf("[SSE] reject reason=busy\n");
      http_send_text_response(connection, "503 Service Unavailable", "text/plain",
                              "SSE busy");
      return false;
    }
  }

  context = (sse_stream_context_t *)pvPortMalloc(sizeof(*context));
  if (context == NULL) {
    http_send_text_response(connection, "500 Internal Server Error", "text/plain",
                            "SSE allocation failed");
    return false;
  }

  *context = (sse_stream_context_t){
      .connection = connection,
      .last_status = {0},
      .has_last_status = false,
      .last_emit_ms = 0u,
  };

  g_sse_active = true;
  g_sse_stop_requested = false;
  if (xTaskCreate(sse_stream_task, "SSETask", 2048u, context,
                  APP_WIFI_TASK_PRIORITY, NULL) != pdPASS) {
    g_sse_active = false;
    vPortFree(context);
    http_send_text_response(connection, "500 Internal Server Error", "text/plain",
                            "SSE task creation failed");
    return false;
  }

  printf("[SSE] task_started\n");
  return true;
}

static bool http_parse_request(struct netconn *connection,
                               http_request_t *out_request) {
  char request_buffer[HTTP_REQUEST_BUFFER_SIZE];
  size_t total_size = 0u;
  size_t header_size = 0u;
  size_t content_length = 0u;
  http_method_t method = HTTP_METHOD_UNKNOWN;
  char path[96];

  if (out_request == NULL) {
    return false;
  }

  memset(out_request, 0, sizeof(*out_request));

  if (!http_receive_request(connection, request_buffer, sizeof(request_buffer),
                            &total_size, &header_size, &content_length)) {
    return false;
  }

  if (!http_parse_request_path_and_method(request_buffer, header_size, &method,
                                          path, sizeof(path))) {
    return false;
  }

  out_request->method = method;
  strncpy(out_request->path, path, sizeof(out_request->path) - 1u);

  if (content_length > 0u) {
    const size_t body_copy_length =
        content_length > HTTP_MAX_BODY_SIZE ? HTTP_MAX_BODY_SIZE : content_length;
    memcpy(out_request->body, request_buffer + header_size, body_copy_length);
    out_request->body[body_copy_length] = '\0';
    out_request->body_length = body_copy_length;

    if (content_length > HTTP_MAX_BODY_SIZE) {
      return false;
    }
  }

  return true;
}

static bool http_handle_status_route(struct netconn *connection,
                                     const http_request_t *request) {
  web_status_snapshot_t status_snapshot = {0};
  char payload[HTTP_RESPONSE_PAYLOAD_BUFFER_SIZE];
  const bool has_snapshot = web_collect_status_snapshot(&status_snapshot);
  const bool payload_ok = has_snapshot &&
                          web_format_status_json(&status_snapshot, payload,
                                                 sizeof(payload));

  if (request->method == HTTP_METHOD_HEAD) {
    http_send_headers_only(connection,
                           payload_ok ? "200 OK" : "500 Internal Server Error",
                           "application/json",
                           payload_ok ? strlen(payload) : 0u);
    return false;
  }

  if (!payload_ok) {
    http_send_text_response(connection, "500 Internal Server Error",
                            "application/json", "{\"error\":\"status\"}");
    return false;
  }

  http_send_response(connection, "200 OK", "application/json",
                     (const uint8_t *)payload, strlen(payload));
  return false;
}

static bool http_handle_test_report_compat_route(struct netconn *connection,
                                                 const http_request_t *request) {
  static const char k_report_payload[] = "{\"active\":false,\"report\":null}";
  static const char k_latest_report_payload[] = "{\"report\":null}";
  const bool is_latest_route =
      strcmp(request->path, "/api/test/report/latest") == 0;
  const char *payload =
      is_latest_route ? k_latest_report_payload : k_report_payload;
  const size_t payload_length = strlen(payload);

  if (request->method == HTTP_METHOD_HEAD) {
    http_send_headers_only(connection, "200 OK", "application/json",
                           payload_length);
    return false;
  }

  http_send_response(connection, "200 OK", "application/json",
                     (const uint8_t *)payload, payload_length);
  return false;
}

static bool http_handle_api_post_route(struct netconn *connection,
                                       const http_request_t *request) {
  int value = 0;
  char response_payload[192];

  if (request->method != HTTP_METHOD_POST) {
    http_send_text_response(connection, "405 Method Not Allowed", "text/plain",
                            "Method Not Allowed");
    return false;
  }

  if (strcmp(request->path, "/api/calibrate") == 0) {
    blower_metrics_snapshot_t metrics_snapshot = {0};
    const bool has_metrics = blower_metrics_service_get_snapshot(&metrics_snapshot);
    int written = 0;

    if (!has_metrics || (!metrics_snapshot.fan_sample_valid &&
                        !metrics_snapshot.envelope_sample_valid)) {
      const char *reason = !has_metrics ? "metrics_unavailable" : "no_valid_samples";
      written = snprintf(
          response_payload, sizeof(response_payload),
          "{\"status\":\"error\",\"reason\":\"%s\",\"fan_ok\":false,\"envelope_ok\":false}",
          reason);
      if (written <= 0 || (size_t)written >= sizeof(response_payload)) {
        http_send_text_response(connection, "500 Internal Server Error",
                                "application/json", "{\"status\":\"error\"}");
        return false;
      }
      http_send_response(connection, "409 Conflict",
                         "application/json", (const uint8_t *)response_payload,
                         strlen(response_payload));
      return false;
    }

    blower_metrics_service_begin_calibration();

    written = snprintf(
        response_payload, sizeof(response_payload),
        "{\"status\":\"ok\",\"reason\":\"calibration_started\","
        "\"fan_ok\":%s,\"envelope_ok\":%s}",
        metrics_snapshot.fan_sample_valid ? "true" : "false",
        metrics_snapshot.envelope_sample_valid ? "true" : "false");

    if (written <= 0 || (size_t)written >= sizeof(response_payload)) {
      http_send_text_response(connection, "500 Internal Server Error",
                              "application/json", "{\"status\":\"error\"}");
      return false;
    }

    http_send_response(connection, "200 OK",
                       "application/json", (const uint8_t *)response_payload,
                       strlen(response_payload));
    return false;
  }

  if (!json_extract_int_field(request->body, "value", &value)) {
    http_send_text_response(connection, "400 Bad Request", "text/plain",
                            "Invalid JSON payload");
    return false;
  }

  if (strcmp(request->path, "/api/pwm") == 0) {
    if (value < 0 || value > 100) {
      http_send_text_response(connection, "400 Bad Request", "text/plain",
                              "PWM value must be between 0 and 100");
      return false;
    }
    blower_control_set_manual_pwm_percent((uint8_t)value);
    debug_logs_append("CMD PWM updated");
  } else if (strcmp(request->path, "/api/led") == 0) {
    if (value != 0 && value != 1) {
      http_send_text_response(connection, "400 Bad Request", "text/plain",
                              "LED value must be 0 or 1");
      return false;
    }
    blower_control_set_auto_hold_enabled(value == 1);
    debug_logs_append(value == 1 ? "CMD AUTO_HOLD ON" : "CMD AUTO_HOLD OFF");
  } else if (strcmp(request->path, "/api/relay") == 0) {
    if (value != 0 && value != 1) {
      http_send_text_response(connection, "400 Bad Request", "text/plain",
                              "Relay value must be 0 or 1");
      return false;
    }
    blower_control_set_relay_enabled(value == 1);
    debug_logs_append(value == 1 ? "CMD RELAY ON" : "CMD RELAY OFF");
  } else {
    http_send_text_response(connection, "404 Not Found", "text/plain",
                            "Not Found");
    return false;
  }

  {
    const int written =
        snprintf(response_payload, sizeof(response_payload),
                 "{\"status\":\"ok\",\"value\":%d}", value);
    if (written <= 0 || (size_t)written >= sizeof(response_payload)) {
      http_send_text_response(connection, "500 Internal Server Error",
                              "application/json", "{\"status\":\"error\"}");
      return false;
    }
  }

  http_send_response(connection, "200 OK", "application/json",
                     (const uint8_t *)response_payload, strlen(response_payload));
  return false;
}

static bool http_handle_debug_route(struct netconn *connection,
                                    const http_request_t *request) {
#if APP_ENABLE_DEBUG_HTTP_ROUTES
  if (strcmp(request->path, "/debug/stream") == 0 &&
      request->method == HTTP_METHOD_POST) {
    bool enabled = false;
    char payload[64];

    if (!json_extract_bool_field(request->body, "enabled", &enabled)) {
      http_send_text_response(connection, "400 Bad Request", "text/plain",
                              "Missing or invalid 'enabled'");
      return false;
    }

    debug_logs_enabled_set(enabled);
    if (!enabled) {
      debug_logs_clear();
    }

    {
      const int written = snprintf(
          payload, sizeof(payload), "{\"status\":\"ok\",\"logs_enabled\":%s}",
          enabled ? "true" : "false");
      if (written <= 0 || (size_t)written >= sizeof(payload)) {
        http_send_text_response(connection, "500 Internal Server Error",
                                "application/json", "{\"status\":\"error\"}");
        return false;
      }
    }

    http_send_response(connection, "200 OK", "application/json",
                       (const uint8_t *)payload, strlen(payload));
    return false;
  }

  if (strcmp(request->path, "/debug/stream") == 0 &&
      request->method == HTTP_METHOD_GET) {
    const bool enabled = debug_logs_enabled_get();
    char payload[64];
    const int written =
        snprintf(payload, sizeof(payload), "{\"logs_enabled\":%s}",
                 enabled ? "true" : "false");
    if (written <= 0 || (size_t)written >= sizeof(payload)) {
      http_send_text_response(connection, "500 Internal Server Error",
                              "application/json", "{\"status\":\"error\"}");
      return false;
    }
    http_send_response(connection, "200 OK", "application/json",
                       (const uint8_t *)payload, strlen(payload));
    return false;
  }

  if (strcmp(request->path, "/debug/clear") == 0 &&
      request->method == HTTP_METHOD_POST) {
    static const char k_ok_payload[] =
        "{\"status\":\"ok\",\"message\":\"Debug buffer cleared\"}";
    debug_logs_clear();
    http_send_response(connection, "200 OK", "application/json",
                       (const uint8_t *)k_ok_payload, sizeof(k_ok_payload) - 1u);
    return false;
  }

  if (strcmp(request->path, "/debug/logs") == 0 &&
      request->method == HTTP_METHOD_GET) {
    char logs[DEBUG_LOG_BUFFER_SIZE];
    debug_logs_copy(logs, sizeof(logs));
    http_send_response(connection, "200 OK", "text/plain; charset=utf-8",
                       (const uint8_t *)logs, strlen(logs));
    return false;
  }

  http_send_text_response(connection, "404 Not Found", "text/plain", "Not Found");
  return false;
#else
  (void)request;
  http_send_text_response(connection, "404 Not Found", "text/plain", "Not Found");
  return false;
#endif
}

static bool http_handle_ota_status_route(struct netconn *connection,
                                         const http_request_t *request) {
  ota_update_status_t status = {0};
  char firmware_version_escaped[96];
  char staged_version_escaped[(OTA_UPDATE_VERSION_LABEL_MAX_LEN * 2u) + 1u];
  char last_error_escaped[(OTA_UPDATE_ERROR_TEXT_MAX_LEN * 2u) + 1u];
  char payload[HTTP_RESPONSE_PAYLOAD_BUFFER_SIZE];
  const char *firmware_version = ota_update_service_get_firmware_version();
  uint32_t progress_percent = 0u;
  int written = 0;

  ota_update_service_get_status(&status);
  progress_percent = status.expected_size_bytes == 0u
                         ? 0u
                         : (status.received_size_bytes * 100u) /
                               status.expected_size_bytes;

  if (!json_escape_string(firmware_version, firmware_version_escaped,
                          sizeof(firmware_version_escaped)) ||
      !json_escape_string(status.staged_version, staged_version_escaped,
                          sizeof(staged_version_escaped)) ||
      !json_escape_string(status.last_error, last_error_escaped,
                          sizeof(last_error_escaped))) {
    http_send_text_response(connection, "500 Internal Server Error",
                            "application/json", "{\"error\":\"ota_status\"}");
    return false;
  }

  written = snprintf(
      payload, sizeof(payload),
      "{\"firmware_version\":\"%s\",\"state\":\"%s\","
      "\"expected_size\":%lu,\"received_size\":%lu,\"progress_percent\":%lu,"
      "\"expected_crc32\":%lu,\"computed_crc32\":%lu,\"staged_version\":\"%s\","
      "\"apply_task_active\":%s,\"last_error\":\"%s\"}",
      firmware_version_escaped, ota_update_service_state_name(status.state),
      (unsigned long)status.expected_size_bytes,
      (unsigned long)status.received_size_bytes, (unsigned long)progress_percent,
      (unsigned long)status.expected_crc32, (unsigned long)status.computed_crc32,
      staged_version_escaped, status.apply_task_active ? "true" : "false",
      last_error_escaped);
  if (written <= 0 || (size_t)written >= sizeof(payload)) {
    http_send_text_response(connection, "500 Internal Server Error",
                            "application/json", "{\"error\":\"ota_payload\"}");
    return false;
  }

  if (request->method == HTTP_METHOD_HEAD) {
    http_send_headers_only(connection, "200 OK", "application/json",
                           strlen(payload));
    return false;
  }

  http_send_response(connection, "200 OK", "application/json",
                     (const uint8_t *)payload, strlen(payload));
  return false;
}

static void http_send_ota_result_response(struct netconn *connection,
                                          const char *status_line,
                                          ota_update_result_t result) {
  char payload[128];
  const int written =
      snprintf(payload, sizeof(payload), "{\"status\":\"%s\"}",
               ota_update_result_name(result));
  if (written <= 0 || (size_t)written >= sizeof(payload)) {
    http_send_text_response(connection, "500 Internal Server Error",
                            "application/json", "{\"status\":\"internal\"}");
    return;
  }

  http_send_response(connection, status_line, "application/json",
                     (const uint8_t *)payload, strlen(payload));
}

static bool http_handle_ota_post_route(struct netconn *connection,
                                       const http_request_t *request) {
  ota_update_result_t result = OTA_UPDATE_RESULT_INVALID_ARGUMENT;

  if (request->method != HTTP_METHOD_POST) {
    http_send_text_response(connection, "405 Method Not Allowed", "text/plain",
                            "Method Not Allowed");
    return false;
  }

  if (strcmp(request->path, "/api/ota/begin") == 0) {
    uint32_t image_size = 0u;
    uint32_t expected_crc32 = 0u;
    char version_label[OTA_UPDATE_VERSION_LABEL_MAX_LEN];

    if (!json_extract_uint32_field(request->body, "size", &image_size) ||
        !json_extract_uint32_field(request->body, "crc32", &expected_crc32)) {
      http_send_text_response(connection, "400 Bad Request", "text/plain",
                              "Missing size or crc32");
      return false;
    }

    version_label[0] = '\0';
    if (!json_extract_string_field(request->body, "version", version_label,
                                   sizeof(version_label))) {
      strcpy(version_label, "unspecified");
    }

    result = ota_update_service_begin(image_size, expected_crc32, version_label);
    if (result == OTA_UPDATE_RESULT_OK) {
      http_send_ota_result_response(connection, "200 OK", result);
      return false;
    }

    http_send_ota_result_response(connection, "400 Bad Request", result);
    return false;
  }

  if (strcmp(request->path, "/api/ota/chunk") == 0) {
    uint32_t offset = 0u;
    char encoded_chunk[HTTP_MAX_BODY_SIZE + 1u];
    size_t decoded_size = 0u;

    if (!json_extract_uint32_field(request->body, "offset", &offset) ||
        !json_extract_string_field(request->body, "data", encoded_chunk,
                                   sizeof(encoded_chunk))) {
      http_send_text_response(connection, "400 Bad Request", "text/plain",
                              "Missing offset or data");
      return false;
    }

    if (!base64_decode_payload(encoded_chunk, g_ota_decoded_chunk_buffer,
                               sizeof(g_ota_decoded_chunk_buffer),
                               &decoded_size) ||
        decoded_size == 0u) {
      http_send_text_response(connection, "400 Bad Request", "text/plain",
                              "Invalid base64 chunk");
      return false;
    }

    result = ota_update_service_write_chunk(offset, g_ota_decoded_chunk_buffer,
                                            decoded_size);
    if (result == OTA_UPDATE_RESULT_OK) {
      http_send_ota_result_response(connection, "200 OK", result);
      return false;
    }

    http_send_ota_result_response(connection, "400 Bad Request", result);
    return false;
  }

  if (strcmp(request->path, "/api/ota/finish") == 0) {
    result = ota_update_service_finish();
    if (result == OTA_UPDATE_RESULT_OK) {
      http_send_ota_result_response(connection, "200 OK", result);
      return false;
    }

    http_send_ota_result_response(connection, "400 Bad Request", result);
    return false;
  }

  if (strcmp(request->path, "/api/ota/apply") == 0) {
    result = ota_update_service_request_apply_async();
    if (result == OTA_UPDATE_RESULT_OK) {
      http_send_ota_result_response(connection, "202 Accepted", result);
      return false;
    }

    http_send_ota_result_response(connection, "409 Conflict", result);
    return false;
  }

  http_send_text_response(connection, "404 Not Found", "text/plain", "Not Found");
  return false;
}

static bool http_path_equals_any(const char *path, const char *const *candidates,
                                 size_t candidates_count) {
  size_t index = 0u;

  if (path == NULL || candidates == NULL) {
    return false;
  }

  for (index = 0u; index < candidates_count; ++index) {
    if (strcmp(path, candidates[index]) == 0) {
      return true;
    }
  }

  return false;
}

static bool http_server_serve_connection(struct netconn *connection) {
  http_request_t request;
  static const char *const k_control_post_routes[] = {"/api/pwm", "/api/led",
                                                       "/api/relay",
                                                       "/api/calibrate"};
  static const char *const k_ota_post_routes[] = {"/api/ota/begin",
                                                   "/api/ota/chunk",
                                                   "/api/ota/finish",
                                                   "/api/ota/apply"};
  bool method_is_get_or_head = false;

  if (!http_parse_request(connection, &request)) {
    http_send_text_response(connection, "400 Bad Request", "text/plain",
                            "Bad Request");
    netconn_close(connection);
    return false;
  }

  method_is_get_or_head = request.method == HTTP_METHOD_GET ||
                          request.method == HTTP_METHOD_HEAD;

  if (strcmp(request.path, "/favicon.ico") == 0) {
    http_send_headers_only(connection, "204 No Content", "image/x-icon", 0u);
    netconn_close(connection);
    return false;
  }

  if (method_is_get_or_head && strcmp(request.path, "/api/status") == 0) {
    (void)http_handle_status_route(connection, &request);
    netconn_close(connection);
    return false;
  }

  if (method_is_get_or_head && strcmp(request.path, "/api/ota/status") == 0) {
    (void)http_handle_ota_status_route(connection, &request);
    netconn_close(connection);
    return false;
  }

  if (method_is_get_or_head &&
      (strcmp(request.path, "/api/test/report") == 0 ||
       strcmp(request.path, "/api/test/report/latest") == 0)) {
    (void)http_handle_test_report_compat_route(connection, &request);
    netconn_close(connection);
    return false;
  }

  if (request.method == HTTP_METHOD_POST &&
      http_path_equals_any(request.path, k_control_post_routes,
                           sizeof(k_control_post_routes) /
                               sizeof(k_control_post_routes[0]))) {
    (void)http_handle_api_post_route(connection, &request);
    netconn_close(connection);
    return false;
  }

  if (request.method == HTTP_METHOD_POST &&
      http_path_equals_any(request.path, k_ota_post_routes,
                           sizeof(k_ota_post_routes) /
                               sizeof(k_ota_post_routes[0]))) {
    (void)http_handle_ota_post_route(connection, &request);
    netconn_close(connection);
    return false;
  }

#if APP_ENABLE_DEBUG_HTTP_ROUTES
  if ((strncmp(request.path, "/debug/", 7u) == 0 ||
       strcmp(request.path, "/debug/stream") == 0) &&
      (request.method == HTTP_METHOD_GET || request.method == HTTP_METHOD_POST)) {
    (void)http_handle_debug_route(connection, &request);
    netconn_close(connection);
    return false;
  }
#endif

  if (request.method == HTTP_METHOD_GET && strcmp(request.path, "/events") == 0) {
    if (http_start_sse_stream(connection)) {
      return true;
    }

    netconn_close(connection);
    return false;
  }

  if (method_is_get_or_head) {
    const char *content_type = NULL;
    const uint8_t *body = NULL;
    size_t body_length = 0u;

    if (web_assets_get(request.path, &content_type, &body, &body_length)) {
      if (request.method == HTTP_METHOD_HEAD) {
        http_send_headers_only(connection, "200 OK", content_type, body_length);
      } else {
        http_send_response(connection, "200 OK", content_type, body, body_length);
      }
    } else {
      http_send_text_response(connection, "404 Not Found", "text/plain",
                              "Not Found");
    }

    netconn_close(connection);
    return false;
  }

  http_send_text_response(connection, "405 Method Not Allowed", "text/plain",
                          "Method Not Allowed");
  netconn_close(connection);
  return false;
}

static void wifi_log_ip_address(void) {
  if (netif_default == NULL) {
    return;
  }

  {
    const char *ip_address = ip4addr_ntoa(netif_ip4_addr(netif_default));
    if (ip_address != NULL) {
      printf("[WiFi] IP: %s\n", ip_address);
    }
  }
}

static bool wifi_connect_station_mode(void) {
  if (cyw43_arch_init() != 0) {
    printf("[WiFi] init failed\n");
    return false;
  }

  cyw43_arch_enable_sta_mode();

  while (cyw43_arch_wifi_connect_timeout_ms(
             WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK,
             WIFI_CONNECT_TIMEOUT_MS) != 0) {
    vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_DELAY_MS));
  }

  cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);
  wifi_log_ip_address();
  return true;
}

static struct netconn *http_server_create_listener(void) {
  struct netconn *listener = netconn_new(NETCONN_TCP);
  err_t status = ERR_OK;

  if (listener == NULL) {
    return NULL;
  }

  status = netconn_bind(listener, IP_ADDR_ANY, HTTP_SERVER_PORT);
  if (status != ERR_OK) {
    netconn_delete(listener);
    return NULL;
  }

  status = netconn_listen_with_backlog(listener, 8u);
  if (status != ERR_OK) {
    netconn_delete(listener);
    return NULL;
  }

  return listener;
}

void wifi_task_entry(void *params) {
  struct netconn *listener = NULL;
  bool led_state = false;

  (void)params;
  debug_logs_clear();
  debug_logs_enabled_set(false);
  ota_update_service_init();

  if (!wifi_connect_station_mode()) {
    vTaskDelete(NULL);
    return;
  }

  listener = http_server_create_listener();
  if (listener == NULL) {
    printf("[WiFi] HTTP init failed\n");
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    struct netconn *client_connection = NULL;
    const err_t accept_status = netconn_accept(listener, &client_connection);

    led_state = !led_state;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);

    if (accept_status == ERR_OK && client_connection != NULL) {
      const bool handed_to_worker =
          http_server_serve_connection(client_connection);
      if (!handed_to_worker) {
        netconn_delete(client_connection);
      }
    }
  }
}
