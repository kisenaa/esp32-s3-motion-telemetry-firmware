#include "console_cli.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* Config Options Used
  - CONFIG_APP_CONSOLE_UART_NUM: UART number for console REPL
  - CONFIG_APP_CONSOLE_UART_BAUD_RATE: UART baud rate for console REPL
  - CONFIG_APP_CONSOLE_MAX_COMMAND_LINE_LENGTH: Maximum command line length
  - CONFIG_APP_CONSOLE_HISTORY_LENGTH: In-memory command history length
  - CONFIG_APP_CONSOLE_TASK_STACK_SIZE: Stack size for console REPL task
  - CONFIG_APP_CONSOLE_TASK_PRIORITY: FreeRTOS priority for console REPL task
*/

static const char *TAG = "console_cli";

static constexpr char CLI_PROMPT[] = "motion> ";
static constexpr int64_t MICROSECONDS_PER_SECOND = 1000000;

static esp_console_repl_t *s_repl = nullptr;
static bool s_initialized = false;
static console_cli_config_t s_runtime;

typedef struct {
  const char *name;
  uint32_t millihz;
} console_cli_rate_t;

static const console_cli_rate_t SUPPORTED_RATES[] = {
    {.name = "800", .millihz = 800000U}, {.name = "400", .millihz = 400000U},
    {.name = "200", .millihz = 200000U}, {.name = "100", .millihz = 100000U},
    {.name = "50", .millihz = 50000U},   {.name = "12.5", .millihz = 12500U},
    {.name = "6.25", .millihz = 6250U},  {.name = "1.56", .millihz = 1560U},
};

static int console_cli_print_usage(const char *usage) {
  printf("usage: %s\n", usage);
  return ESP_ERR_INVALID_ARG;
}

static esp_err_t console_cli_get_state(console_cli_runtime_state_t *state) {
  if (s_runtime.get_state == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  return s_runtime.get_state(s_runtime.context, state);
}

static void console_cli_print_rate(uint32_t millihz) {
  const uint32_t whole = millihz / 1000U;
  const uint32_t fraction = millihz % 1000U;

  if (fraction == 0U) {
    printf("%" PRIu32 " Hz", whole);
    return;
  }

  printf("%" PRIu32 ".%03" PRIu32 " Hz", whole, fraction);
}

static int console_cli_status_command(int argc, char **argv) {
  (void)argv;

  if (argc != 1) {
    return console_cli_print_usage("status");
  }

  console_cli_runtime_state_t state;
  const esp_err_t err = console_cli_get_state(&state);

  if (err != ESP_OK) {
    printf("error: unable to read application state: %s\n",
           esp_err_to_name(err));
    return err;
  }

  printf("sensor: %s\n", state.sensor_ready ? "online" : "offline");
  printf("sd: %s\n", state.storage_ready ? "mounted" : "unavailable");
  printf("logging: %s\n", state.logging_enabled ? "enabled" : "disabled");
  printf("sample_rate: ");
  console_cli_print_rate(state.sample_rate_millihz);
  printf("\nqueue: %" PRIu32 "/%" PRIu32 "\n", state.queued_samples,
         state.queue_capacity);
  printf("uptime: %lld s\n",
         (long long)(esp_timer_get_time() / MICROSECONDS_PER_SECOND));

  return ESP_OK;
}

static int console_cli_read_command(int argc, char **argv) {
  (void)argv;

  if (argc != 1) {
    return console_cli_print_usage("read");
  }

  console_cli_runtime_state_t state;
  const esp_err_t err = console_cli_get_state(&state);

  if (err != ESP_OK) {
    printf("error: unable to read application state: %s\n",
           esp_err_to_name(err));
    return err;
  }

  if (!state.latest_sample_valid) {
    printf("sample: unavailable\n");
    return ESP_OK;
  }

  printf("raw: x=%" PRId16 " y=%" PRId16 " z=%" PRId16 "\n", state.x_raw,
         state.y_raw, state.z_raw);

  printf("g: x=%.6f y=%.6f z=%.6f\n", (double)state.x_g, (double)state.y_g,
         (double)state.z_g);

  printf("timestamp: %" PRId64 " us\n", state.timestamp_us);
  return ESP_OK;
}

static int console_cli_log_command(int argc, char **argv) {
  if (argc != 2) {
    return console_cli_print_usage("log <on|off>");
  }

  bool enable;

  if (strcmp(argv[1], "on") == 0) {
    enable = true;
  } else if (strcmp(argv[1], "off") == 0) {
    enable = false;

  } else {
    return console_cli_print_usage("log <on|off>");
  }

  if (s_runtime.set_logging == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  const esp_err_t err = s_runtime.set_logging(s_runtime.context, enable);

  if (err != ESP_OK) {
    printf("error: logging change failed: %s\n", esp_err_to_name(err));
    return err;
  }

  printf("logging %s\n", enable ? "enabled" : "disabled");
  return ESP_OK;
}

static const console_cli_rate_t *console_cli_find_rate(const char *name) {
  for (size_t index = 0;
       index < sizeof(SUPPORTED_RATES) / sizeof(SUPPORTED_RATES[0]); ++index) {
    if (strcmp(name, SUPPORTED_RATES[index].name) == 0) {
      return &SUPPORTED_RATES[index];
    }
  }

  return nullptr;
}

static int console_cli_rate_command(int argc, char **argv) {
  if (argc != 2) {
    return console_cli_print_usage("rate <hz>");
  }

  const console_cli_rate_t *rate = console_cli_find_rate(argv[1]);

  if (rate == nullptr) {
    printf("error: unsupported rate '%s'\n", argv[1]);

    printf("supported rates: 800 400 200 100 50 "
           "12.5 6.25 1.56\n");
    return ESP_ERR_INVALID_ARG;
  }

  if (s_runtime.set_rate == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  const esp_err_t err = s_runtime.set_rate(s_runtime.context, rate->millihz);
  if (err != ESP_OK) {
    printf("error: rate change failed: %s\n", esp_err_to_name(err));
    return err;
  }

  printf("sample rate: %s Hz\n", rate->name);
  return ESP_OK;
}

static int console_cli_stats_command(int argc, char **argv) {
  (void)argv;

  if (argc != 1) {
    return console_cli_print_usage("stats");
  }

  console_cli_runtime_state_t state;
  const esp_err_t err = console_cli_get_state(&state);
  if (err != ESP_OK) {
    printf("error: unable to read application state: %s\n",
           esp_err_to_name(err));
    return err;
  }

  printf("samples_acquired: %" PRIu64 "\n", state.samples_acquired);
  printf("sensor_interrupts: %" PRIu64 "\n", state.sensor_interrupts);

  printf("samples_logged:   %" PRIu64 "\n", state.samples_logged);
  printf("queue_drops:      %" PRIu64 "\n", state.queue_drops);
  printf("i2c_errors:       %" PRIu64 "\n", state.i2c_errors);
  printf("sd_errors:        %" PRIu64 "\n", state.sd_errors);

  return ESP_OK;
}

static const esp_console_cmd_t s_commands[] = {
    {
        .command = "status",
        .help = "Show the current high-level firmware state",
        .hint = nullptr,
        .func = console_cli_status_command,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context = nullptr,
    },
    {
        .command = "read",
        .help = "Show the latest cached acceleration sample",
        .hint = nullptr,
        .func = console_cli_read_command,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context = nullptr,
    },
    {
        .command = "log",
        .help = "Enable or disable SD logging",
        .hint = "<on|off>",
        .func = console_cli_log_command,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context = nullptr,
    },
    {
        .command = "rate",
        .help = "Set the MMA8451Q output data rate",
        .hint = "<hz>",
        .func = console_cli_rate_command,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context = nullptr,
    },
    {
        .command = "stats",
        .help = "Show cumulative runtime counters",
        .hint = nullptr,
        .func = console_cli_stats_command,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context = nullptr,
    },
};

static constexpr size_t COMMAND_COUNT =
    sizeof(s_commands) / sizeof(s_commands[0]);

static esp_err_t console_cli_register_commands(void) {
  for (size_t index = 0; index < COMMAND_COUNT; ++index) {
    esp_err_t err = esp_console_cmd_register(&s_commands[index]);

    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to register '%s': %s", s_commands[index].command,
               esp_err_to_name(err));
      return err;
    }
  }

  return ESP_OK;
}

static esp_err_t console_cli_create_repl(void) {
  esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

  repl_config.prompt = CLI_PROMPT;
  repl_config.max_history_len = CONFIG_APP_CONSOLE_HISTORY_LENGTH;
  repl_config.max_cmdline_length = CONFIG_APP_CONSOLE_MAX_COMMAND_LINE_LENGTH;
  repl_config.task_stack_size = CONFIG_APP_CONSOLE_TASK_STACK_SIZE;
  repl_config.task_priority = CONFIG_APP_CONSOLE_TASK_PRIORITY;
  repl_config.history_save_path = nullptr;

  esp_console_dev_uart_config_t uart_config =
      ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

  uart_config.channel = CONFIG_APP_CONSOLE_UART_NUM;
  uart_config.baud_rate = CONFIG_APP_CONSOLE_UART_BAUD_RATE;
  uart_config.tx_gpio_num = -1;
  uart_config.rx_gpio_num = -1;

  return esp_console_new_repl_uart(&uart_config, &repl_config, &s_repl);
}

static bool console_cli_config_is_valid(const console_cli_config_t *config) {
  return config != nullptr && config->get_state != nullptr &&
         config->set_logging != nullptr && config->set_rate != nullptr;
}

esp_err_t console_cli_init(const console_cli_config_t *config) {
  if (!console_cli_config_is_valid(config)) {
    return ESP_ERR_INVALID_ARG;
  }

  if (s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  s_runtime = *config;
  esp_err_t err = console_cli_create_repl();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create UART REPL: %s", esp_err_to_name(err));
    memset(&s_runtime, 0, sizeof(s_runtime));
    return err;
  }

  err = esp_console_register_help_command();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register help command: %s", esp_err_to_name(err));
    memset(&s_runtime, 0, sizeof(s_runtime));
    (void)s_repl->del(s_repl);
    s_repl = nullptr;
    return err;
  }

  err = console_cli_register_commands();

  if (err != ESP_OK) {
    memset(&s_runtime, 0, sizeof(s_runtime));
    (void)s_repl->del(s_repl);
    s_repl = nullptr;
    return err;
  }

  err = esp_console_start_repl(s_repl);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start console REPL: %s", esp_err_to_name(err));
    memset(&s_runtime, 0, sizeof(s_runtime));
    (void)s_repl->del(s_repl);
    s_repl = nullptr;
    return err;
  }

  s_initialized = true;

  ESP_LOGI(TAG, "Console CLI started on UART%d at %d baud",
           CONFIG_APP_CONSOLE_UART_NUM, CONFIG_APP_CONSOLE_UART_BAUD_RATE);

  return ESP_OK;
}
