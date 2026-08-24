# ESP32-S3 Motion Telemetry & Data Logger

ESP-IDF 6.0.2 firmware for an ESP32-S3-DevKitC-1 with an MMA8451Q
accelerometer and an SDSPI microSD card. It acquires samples from the
sensor's DATA_READY interrupt, writes optional indexed CSV sessions, and
provides runtime control through a UART REPL.

| | |
| :---: | :---: |
| <video src="https://github.com/user-attachments/assets/07451c9f-f951-4f3a-a432-02f33cbec28d" width="50%"></video> | <img src="https://github.com/user-attachments/assets/93fd87bc-9ac0-4bd7-ad8b-949be9526ed9" width="100%" alt="image" /> |

## Table of Contents

1. [Hardware and Wiring](#hardware-and-wiring)
2. [High-level architecture](#high-level-architecture)
3. [Runtime architecture](#runtime-architecture)
4. [Synchronized state](#synchronized-state)
5. [CLI Commands](#cli)
6. [Code organization](#code-organization)
7. [Configuration and builds](#configuration-and-builds)
8. [Data format](#data-format)
9. [License](#license--usage)

## Hardware and Wiring

Required hardware:
| Hardware | Purpose |
| --------------------------------- | ------------------------------------------------------------------------------- |
| ESP32-S3 DevKitC-1 | Main MCU |
| GY-45 MMA8451Q | 3-axis accelerometer |
| microSD SPI module + card | Persistent telemetry logging |
| Onboard RGB LED | System status |
| External LED + 220–330 Ω resistor | Activity pulse |
| USB data cable | Flashing, monitor, and UART console |
| Breadboard + Dupont wires | Prototyping |

### MMA8451Q wiring

```text
ESP32-S3                  GY-45 / MMA8451Q
─────────                 ────────────────
3V3      ───────────────► VCC / 3V3
GND      ───────────────► GND
GPIO1    ───────────────► SCL
GPIO2    ◄──────────────► SDA
GPIO42   ◄─────────────── INT1
```

The MMA8451Q supports I2C addresses `0x1C` and `0x1D` depending on the SA0 pin.

### microSD wiring

```text
ESP32-S3                  microSD SPI module
─────────                 ──────────────────
GPIO10   ───────────────► CS
GPIO11   ───────────────► MOSI
GPIO12   ───────────────► SCLK / CLK
GPIO13   ◄─────────────── MISO
GND      ───────────────► GND
```

For **VCC**, check whether the exact SD breakout requires 3.3 V or 5 V.

### External LED

```text
GPIO8 ── 220–330 Ω ──► LED anode
                       LED cathode ──► GND
```

## High-level architecture

### Hardware architecture

```text
                              PC
                              │
                         UART console / USB
                              │
                              ▼
                   ┌─────────────────────┐
                   │     ESP32-S3        │
                   │                     │
                   │     ESP-IDF         │
                   │     FreeRTOS        │
                   └──┬──────┬───────┬──┘
                      │      │       │
                    I2C    GPIO     SPI
                      │      │       │
                      ▼      │       ▼
                 ┌────────┐  │   ┌─────────┐
                 │MMA8451Q│──┘   │ microSD │
                 │        │ INT1 │ module  │
                 └────────┘      └─────────┘
                      │
                  X/Y/Z motion
```

### Software architecture

```text
MMA8451Q INT1
   │
  └── GPIO IRQ ──► sensor_task ── I2C ──► MMA8451Q
                 │
                 └── sample queue ──► logger_task ── SPI ──► microSD

PC
 │
 └── UART ──► ESP-IDF console ──► main adapters
                          │
                          ├── app-state snapshot
                          ├── sensor ODR request
                          └── logger control request
```

## Runtime architecture

```text
MMA8451Q DATA_READY
        │
        ▼
 GPIO interrupt
        │
        ▼
 short ISR
        │
        │ vTaskNotifyGiveFromISR()
        ▼
 ┌────────────────┐
 │  sensor_task   │
 └───────┬────────┘
         │
         │ I2C read X/Y/Z
         ▼
  mma8451_sample_t
         │
         │ xQueueSend(..., 0)
         ▼
 ┌────────────────┐
 │  sample_queue  │
 └───────┬────────┘
         │
         │ xQueueReceive()
         ▼
 ┌────────────────┐
 │  logger_task   │
 └───────┬────────┘
         │
         │ CSV append
         ▼
 FATFS → SDSPI → microSD


 PC console ──► main callback adapter ──► logger control queue
                                            │
                                            ▼
                                      logger_task performs SD work
```

- `sensor_task` waits for INT1, reads acceleration over I2C, updates
  synchronized state, and sends samples without waiting for storage. A full
  queue drops the newest sample and records the drop.
- `logger_task` is the only runtime owner of the SD card. It appends, flushes,
  starts/stops sessions, and performs file management requests.
- The ESP-IDF console serves commands while acquisition and logging continue.
  It reads cached state, routes ODR changes to `sensor_task`, and routes SD
  work to `logger_task`; it does not access I2C or the SD card directly.
- `log off` stops CSV writes only. Sampling, the cached sample, counters, and
  CLI remain available.

The ISR only notifies `sensor_task`; it does not perform I2C, storage, or
console work. A sensor or SD failure sets the RGB LED red while leaving the
other subsystem and the CLI available when possible.

RGB status is yellow during startup, green when ready, blue while logging,
and red after a sensor or storage error.

## Synchronized state

- **Application state mutex:** protects the latest sample, availability flags,
  rate, and counters. Callers receive a copied snapshot and never hold the
  mutex during I2C, SD, LED, console, queue, or delay work.
- **Sensor mutex:** serializes MMA8451Q register access between
  interrupt-driven reads and a CLI `rate` request. The GPIO ISR never takes it.
- **Task notification:** the GPIO ISR uses `vTaskNotifyGiveFromISR()` to wake
  `sensor_task`. The task blocks with `ulTaskNotifyTake()` until data is ready,
  a timeout check is needed, or shutdown is requested.
- **Sample queue:** transfers `app_sample_t` values from `sensor_task` to
  `logger_task`. The producer uses a zero wait; storage never delays sampling.
- **Logger control mutex and queues:** serialize one CLI request at a time.
  The command queue wakes `logger_task`, and the result queue returns only an
  `esp_err_t`; list and tail results are written to the waiting caller's buffer.
- **Logger queue set:** lets `logger_task` wait efficiently for either a
  sample or a control request, then perform all SD work in one task context.

### Lifecycle

1. `app_state_init()` creates the static state mutex before tasks start.
2. `logger_task_start()` creates its control mutex, control/result queues, and
   queue set; `sensor_task_start()` creates its mutex, configures GPIO, creates
   the task, then enables DATA_READY delivery.
3. On shutdown, sensor delivery is disabled and the sensor task acknowledges
   its stop request. The logger task acknowledges its stop request before its
   queues and mutex are deleted.
4. The sample queue is deleted after both tasks stop, then `app_state_deinit()`
   deletes the shared-state mutex. Runtime SD cleanup remains in the logger
   ownership path.

## CLI

| Command         | Description                                                        |
| --------------- | ------------------------------------------------------------------ |
| `help`          | Show available commands                                            |
| `status`        | Show current sensor, SD, logging, and queue state                  |
| `read`          | Show the latest cached sample                                      |
| `log on`        | Start the next indexed CSV session                                 |
| `log off`       | Flush and close the current session                                |
| `rate <hz>`     | Set `800`, `400`, `200`, `100`, `50`, `12.5`, `6.25`, or `1.56` Hz |
| `stats`         | Show sample, interrupt, drop, and error counters                   |
| `list`          | List the five newest telemetry files                               |
| `tail <NNNN>`   | Show the last ten lines of one telemetry file                      |
| `delete <NNNN>` | Delete one telemetry file                                          |
| `delete all`    | Delete all indexed telemetry files                                 |

File-management commands operate only on `telemetry_0001.csv` through
`telemetry_9999.csv`, and can only be used while logging is off.

## Code organization

```text
main/
  main.c          startup orchestration and CLI adapters
  app_state.c     mutex-protected runtime state/counters
  sensor_task.c   interrupt acquisition, I2C serialization
  logger_task.c   sample consumer and sole runtime SD owner
components/
  mma8451q/       custom MMA8451Q protocol and driver lifecycle
  sd_card/        SDSPI/FATFS, CSV lifecycle, indexed file operations
  console_cli/    ESP-IDF REPL, parsing, user-facing output
  status_led/     RGB state and activity indication
managed_components/
  espressif__led_strip/ generated dependency; do not edit
```

## Configuration and builds

Pins, rates, task sizes, queue length, recovery settings, and flush policy are in
[main/Kconfig.projbuild](main/Kconfig.projbuild). Development defaults are in
[sdkconfig.defaults](sdkconfig.defaults); the `prod` preset also reads
[sdkconfig.defaults.prod](sdkconfig.defaults.prod).

The project uses C23 features supported by ESP-IDF 6.0.2. Build, flash, and
monitor it through the VS Code ESP-IDF extension. The target is ESP32-S3;
the default console is UART0 at 115200 baud.

## Data format

Each logging session creates the next available `telemetry_NNNN.csv` file:

```csv
timestamp_us,x_raw,y_raw,z_raw,x_g,y_g,z_g
12534120,21,-42,4071,0.005127,-0.010254,0.993896
```

9. License
## License / Usage
Copyright © 2026 Kisenaa. All rights reserved.

Use of this repository, source code, documentation, or derived data
for training, fine-tuning, evaluation, or development of machine
learning or generative AI models is not permitted without prior
written permission.

This project is licensed under the BSD 3-Clause License with additional terms. See [LICENSE](LICENSE) for details.
