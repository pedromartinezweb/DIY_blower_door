# Project Context for LLMs

## Objective

This repository contains firmware for a DIY blower door platform on Raspberry Pi Pico 2 W (RP2350), with:

- 2x ADP910 differential pressure sensors over I2C
- dimmer-based fan power control using zero-cross detection
- embedded web UI over Wi-Fi
- real-time telemetry over SSE
- OTA firmware updates

Primary use case is airtightness testing workflows (n50/n75 style operation).

## Build Source of Truth

The active firmware target is defined in `CMakeLists.txt` as `blower_pico_c`.

Files compiled into the target:

- `src/main.c`
- `src/app/task_bootstrap.c`
- `src/platform/runtime_faults.c`
- `src/drivers/adp910/adp910_sensor.c`
- `src/services/blower_metrics.c`
- `src/services/blower_control.c`
- `src/services/ota_update_service.c`
- `src/services/dimmer_control.c`
- `src/tasks/wifi_task.c`
- `src/tasks/dimmer_task.c`
- `src/tasks/adp910_task.c`
- generated web bundle: `build/generated/web_assets.c`

Important: treat `CMakeLists.txt` as the ground truth of what is active. There are legacy files in the repo that are not part of this build.

## Runtime Architecture

`src/main.c` starts FreeRTOS and creates default tasks through `src/app/task_bootstrap.c`.

Current FreeRTOS tasks:

- `WiFiTask` (`src/tasks/wifi_task.c`)
- `DimmerTask` (`src/tasks/dimmer_task.c`)
- `ADP910Task` (`src/tasks/adp910_task.c`)

Task enable flags, priorities, and most runtime tuning are configured in `include/app/app_config.h`.

## Hardware Mapping (Current Build)

ADP910 mapping is configurable in `include/app/app_config.h`:

- Fan sensor: `i2c0` on GPIO4/5
- Envelope sensor: `i2c1` on GPIO6/7
- ADP910 address: `0x25`
- I2C default frequency: `100000` Hz

Dimmer mapping in active code is currently in `src/tasks/dimmer_task.c`:

- zero-cross input: GPIO2
- gate output: GPIO3

## ADP910 Integration

Driver: `src/drivers/adp910/adp910_sensor.c`.

Protocol details implemented:

- continuous measurement command: `0x361E`
- 6-byte read frame
- CRC8 polynomial `0x31`, init `0xFF`
- pressure conversion: `raw / 60` (Pa)
- temperature conversion: `raw / 200` (C)

Sampling task: `src/tasks/adp910_task.c` initializes both sensors, retries on failure, and updates shared metrics in `src/services/blower_metrics.c`.

## Fan Control Path

- `src/services/blower_control.c` contains manual and pressure-hold control logic.
- `src/tasks/dimmer_task.c` runs the loop, reads metrics, computes output percent, and drives triac firing timing via GPIO IRQ + timer alarms.
- `src/services/dimmer_control.c` stores current power percent shared between task logic and ISR paths.

## Web/API and SSE

HTTP server and routing are implemented directly in `src/tasks/wifi_task.c`.

Main routes in active firmware:

- `GET /` static web UI
- `GET /events` SSE telemetry stream
- `GET /api/status`
- `POST /api/pwm` with `{"value":0..100}`
- `POST /api/led` with `{"value":0|1}` (auto hold)
- `POST /api/relay` with `{"value":0|1}`
- `POST /api/calibrate` (zero offsets in metrics service)
- `GET /api/ota/status`
- `POST /api/ota/begin`
- `POST /api/ota/chunk`
- `POST /api/ota/finish`
- `POST /api/ota/apply`

Compatibility route:

- `GET /api/test/report` and `GET /api/test/report/latest` currently return placeholder payloads.

SSE behavior:

- one active SSE client at a time
- reconnect handover logic if a new client connects
- periodic forced publish plus change-based publish

## Frontend Structure

Two web folders exist:

- `include/web/`: full production UI used by default in current builds
- `web/`: older minimal UI

Default embedding order in CMake prefers `include/web/` when present.

Frontend logic in `include/web/app.js`:

- consumes SSE from `/events`
- sends control commands (`/api/pwm`, `/api/led`, `/api/relay`, `/api/calibrate`)
- handles OTA upload flow
- computes ACH-style indicators client-side from telemetry and local settings

## OTA Flow

OTA service: `src/services/ota_update_service.c`.

Features implemented:

- staging area in flash
- chunked upload with CRC32 verification
- vector table sanity checks before apply
- async apply task and reboot

## Legacy / Not Compiled by Default

The following modules exist but are not compiled in `blower_pico_c`:

- `src/core0/*`
- `src/core1/*`
- `src/shared/*`
- `src/services/blower_test_service.c`
- `src/services/web_status_service.c`
- `src/services/http_payload_utils.c`
- `src/services/http_server_common.c`
- `src/services/debug_logs.c`

Related headers may still exist and can cause confusion if you do not verify against `CMakeLists.txt`.

## Practical Guidance for Edits

When changing behavior:

- task wiring and active modules: edit `CMakeLists.txt` first
- HTTP/SSE API: edit `src/tasks/wifi_task.c` and `include/web/app.js`
- ADP910 behavior: edit `src/drivers/adp910/adp910_sensor.c` and `src/tasks/adp910_task.c`
- control loop behavior: edit `src/services/blower_control.c` and `src/tasks/dimmer_task.c`
- tuning constants: edit `include/app/app_config.h`

If you need multicore dedicated dimmer execution, reuse `src/core1/dimmer_core1.c` as reference, but it is not active in the current build.
