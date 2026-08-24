# ESP32-S3 Motion Telemetry & Data Logger

ESP-IDF 6.0.2 firmware for an ESP32-S3-DevKitC-1 with an MMA8451Q
accelerometer and an SDSPI microSD card. The firmware performs
interrupt-driven acquisition, queues samples between FreeRTOS tasks, logs
indexed CSV sessions, and exposes runtime control over a UART REPL.

## Hardware

| Function              | GPIO |
| --------------------- | ---: |
| MMA8451Q SCL          |    1 |
| MMA8451Q SDA          |    2 |
| MMA8451Q INT1         |   42 |
| External activity LED |    8 |
| SD CS                 |   10 |
| SD MOSI               |   11 |
| SD SCLK               |   12 |
| SD MISO               |   13 |
| Onboard RGB LED       |   48 |

The MMA8451Q interrupt is active-low and push-pull. All modules must share
ground. Confirm the supply voltage required by the exact SD breakout.

## Runtime architecture

- `sensor_task` waits for INT1, reads acceleration over I2C, updates
  synchronized state, and sends samples without blocking on storage.
- `logger_task` owns runtime CSV append, flush, and session changes.
- The ESP-IDF console task serves `status`, `read`, `log`, `rate`, and `stats`
  without directly accessing I2C or the SD card.

RGB states are yellow while initializing, green when ready with logging off,
blue while logging, and red after a hardware or storage error.

## CLI

| Command     | Purpose                                                        |
| ----------- | -------------------------------------------------------------- |
| `help`      | List commands and syntax                                       |
| `status`    | Show synchronized device and queue state                       |
| `read`      | Show the latest cached sample                                  |
| `log on`    | Create and start the next indexed CSV session                  |
| `log off`   | Flush and close the current session                            |
| `rate <hz>` | Set 800, 400, 200, 100, 50, 12.5, 6.25, or 1.56 Hz             |
| `stats`     | Show acquisition, logging, drop, interrupt, and error counters |

Logging is disabled by default; `log on` creates the first indexed session.
Acquisition continues when logging is disabled or storage is unavailable.

## Configuration and builds

Project pins, frequencies, task sizes, queue length, and flush policy are in
[main/Kconfig.projbuild](main/Kconfig.projbuild). Development defaults are in
[sdkconfig.defaults](sdkconfig.defaults); the `prod` preset also reads
[sdkconfig.defaults.prod](sdkconfig.defaults.prod).

The project uses C23 features supported by ESP-IDF 6.0.2. Use the VS Code
ESP-IDF extension to build, flash, and monitor the `default` or `prod` preset.

## Data format

Each logging session creates the next available `telemetry_NNNN.csv` file:

```csv
timestamp_us,x_raw,y_raw,z_raw,x_g,y_g,z_g
12534120,21,-42,4071,0.005127,-0.010254,0.993896
```

See [docs/architecture.md](docs/architecture.md) for the full architecture,
ownership boundaries, wiring notes, roadmap, and references.
