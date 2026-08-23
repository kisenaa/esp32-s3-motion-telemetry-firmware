#include "console_cli.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
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

static int console_cli_print_usage(const char *usage) {
  printf("usage: %s\n", usage);
  return ESP_ERR_INVALID_ARG;
}

static int console_cli_status_command(int argc, char **argv) {
  (void)argv;

  if (argc != 1) {
    return console_cli_print_usage("status");
  }

  const int64_t uptime_seconds = esp_timer_get_time() / MICROSECONDS_PER_SECOND;

  printf("firmware: running\n");
  printf("console: online\n");
  printf("sensor: not initialized\n");
  printf("sd: not initialized\n");
  printf("logging: unavailable\n");
  printf("uptime: %lld s\n", (long long)uptime_seconds);

  return ESP_OK;
}

static int console_cli_read_command(int argc, char **argv) {
  (void)argv;

  if (argc != 1) {
    return console_cli_print_usage("read");
  }

  printf("read: unavailable; motion acquisition is not initialized\n");
  return ESP_OK;
}

static int console_cli_log_command(int argc, char **argv) {
  if (argc != 2) {
    return console_cli_print_usage("log <on|off>");
  }

  if (strcmp(argv[1], "on") == 0) {
    printf("log on: unavailable; SD logging is not initialized\n");
    return ESP_OK;
  }

  if (strcmp(argv[1], "off") == 0) {
    printf("log off: unavailable; SD logging is not initialized\n");
    return ESP_OK;
  }

  return console_cli_print_usage("log <on|off>");
}

static bool console_cli_is_supported_rate(const char *rate) {
  static const char *const supported_rates[] = {
      "800", "400", "200", "100", "50", "12.5", "6.25", "1.56",
  };

  for (size_t index = 0;
       index < sizeof(supported_rates) / sizeof(supported_rates[0]); ++index) {
    if (strcmp(rate, supported_rates[index]) == 0) {
      return true;
    }
  }

  return false;
}

static int console_cli_rate_command(int argc, char **argv) {
  if (argc != 2) {
    return console_cli_print_usage("rate <hz>");
  }

  if (!console_cli_is_supported_rate(argv[1])) {
    printf("error: unsupported rate '%s'\n", argv[1]);
    printf("supported rates: 800 400 200 100 50 12.5 6.25 1.56\n");
    return ESP_ERR_INVALID_ARG;
  }

  printf("rate %s Hz: unavailable; motion sensor is not initialized\n",
         argv[1]);

  return ESP_OK;
}

static int console_cli_stats_command(int argc, char **argv) {
  (void)argv;

  if (argc != 1) {
    return console_cli_print_usage("stats");
  }

  printf("samples_acquired: unavailable\n");
  printf("samples_logged:   unavailable\n");
  printf("queue_drops:      unavailable\n");
  printf("i2c_errors:       unavailable\n");
  printf("sd_errors:        unavailable\n");
  printf("interrupts:       unavailable\n");

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

esp_err_t console_cli_init(void) {
  if (s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t err = console_cli_create_repl();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create UART REPL: %s", esp_err_to_name(err));
    return err;
  }

  err = esp_console_register_help_command();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register help command: %s", esp_err_to_name(err));
    (void)s_repl->del(s_repl);
    s_repl = nullptr;
    return err;
  }

  err = console_cli_register_commands();

  if (err != ESP_OK) {
    (void)s_repl->del(s_repl);
    s_repl = nullptr;
    return err;
  }

  err = esp_console_start_repl(s_repl);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start console REPL: %s", esp_err_to_name(err));
    (void)s_repl->del(s_repl);
    s_repl = nullptr;
    return err;
  }

  s_initialized = true;

  ESP_LOGI(TAG, "Console CLI started on UART%d at %d baud",
           CONFIG_APP_CONSOLE_UART_NUM, CONFIG_APP_CONSOLE_UART_BAUD_RATE);

  return ESP_OK;
}
