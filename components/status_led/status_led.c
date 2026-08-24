#include "status_led.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "led_strip_rmt.h"
#include "sdkconfig.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Pin Mapping Used
  - CONFIG_APP_STATUS_RGB_LED_GPIO: (WS2812 RGB LED)
  - CONFIG_APP_STATUS_ACTIVITY_LED_GPIO:(External Activity LED)
*/

static const char *const TAG = "status_led";

static constexpr uint32_t RGB_LED_COUNT = 1U;
static constexpr uint32_t RGB_LED_INDEX = 0U;
static constexpr uint32_t RGB_LED_RMT_RESOLUTION_HZ = 10U * 1000U * 1000U;
static constexpr uint64_t ACTIVITY_PULSE_DURATION_US = (uint64_t)(200U * 1000U);

static constexpr uint8_t COLOR_OFF = 0U;
static constexpr uint8_t COLOR_LOW = 16U;

typedef struct {
  led_strip_handle_t rgb_led;
  esp_timer_handle_t activity_timer;
  status_led_state_t state;
  bool initialized;
} status_led_context_t;

static status_led_context_t s_status_led = {
    .rgb_led = nullptr,
    .activity_timer = nullptr,
    .state = STATUS_LED_OFF,
    .initialized = false,
};

static esp_err_t status_led_apply_rgb(status_led_state_t state) {
  uint8_t red = COLOR_OFF;
  uint8_t green = COLOR_OFF;
  uint8_t blue = COLOR_OFF;

  switch (state) {
  case STATUS_LED_OFF:
    return led_strip_clear(s_status_led.rgb_led);

  case STATUS_LED_INITIALIZING:
    red = COLOR_LOW;
    green = COLOR_LOW;
    break;

  case STATUS_LED_READY:
    green = COLOR_LOW;
    break;

  case STATUS_LED_LOGGING:
    blue = COLOR_LOW;
    break;

  case STATUS_LED_ERROR:
    red = COLOR_LOW;
    break;

  default:
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t err = led_strip_set_pixel(s_status_led.rgb_led, RGB_LED_INDEX, red,
                                      green, blue);

  if (err != ESP_OK) {
    return err;
  }

  return led_strip_refresh(s_status_led.rgb_led);
}

static void status_led_activity_timer_callback(void *arg) {
  (void)arg;
  (void)gpio_set_level(CONFIG_APP_STATUS_ACTIVITY_LED_GPIO, 0);
}

static esp_err_t status_led_configure_activity_gpio(void) {
  const gpio_config_t gpio_io_config = {
      .pin_bit_mask = 1ULL << CONFIG_APP_STATUS_ACTIVITY_LED_GPIO,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t err =
      gpio_reset_pin((gpio_num_t)CONFIG_APP_STATUS_ACTIVITY_LED_GPIO);

  if (err != ESP_OK) {
    return err;
  }

  err = gpio_config(&gpio_io_config);

  if (err != ESP_OK) {
    return err;
  }

  return gpio_set_level(CONFIG_APP_STATUS_ACTIVITY_LED_GPIO, 0);
}

static esp_err_t status_led_configure_rgb(void) {
  const led_strip_config_t strip_config = {
      .strip_gpio_num = CONFIG_APP_STATUS_RGB_LED_GPIO,
      .max_leds = RGB_LED_COUNT,
      .led_model = LED_MODEL_WS2812,
      .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
      .flags =
          {
              .invert_out = false,
          },
  };

  const led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = RGB_LED_RMT_RESOLUTION_HZ,
      .mem_block_symbols = 0,
      .flags =
          {
              .with_dma = false,
          },
  };

  esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config,
                                           &s_status_led.rgb_led);

  if (err != ESP_OK) {
    return err;
  }

  return led_strip_clear(s_status_led.rgb_led);
}

static esp_err_t status_led_create_activity_timer(void) {
  const esp_timer_create_args_t timer_config = {
      .callback = status_led_activity_timer_callback,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "status_led_pulse",
      .skip_unhandled_events = true,
  };

  return esp_timer_create(&timer_config, &s_status_led.activity_timer);
}

static void status_led_cleanup(void) {
  if (s_status_led.activity_timer != nullptr) {
    if (esp_timer_is_active(s_status_led.activity_timer)) {
      (void)esp_timer_stop(s_status_led.activity_timer);
    }

    (void)esp_timer_delete(s_status_led.activity_timer);
    s_status_led.activity_timer = nullptr;
  }

  if (s_status_led.rgb_led != nullptr) {
    (void)led_strip_clear(s_status_led.rgb_led);
    (void)led_strip_del(s_status_led.rgb_led);
    s_status_led.rgb_led = nullptr;
  }

  (void)gpio_set_level(CONFIG_APP_STATUS_ACTIVITY_LED_GPIO, 0);

  s_status_led.state = STATUS_LED_OFF;
  s_status_led.initialized = false;
}

esp_err_t status_led_init(void) {
  if (s_status_led.initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t err = status_led_configure_activity_gpio();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure activity GPIO: %s",
             esp_err_to_name(err));
    status_led_cleanup();
    return err;
  }

  err = status_led_configure_rgb();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure RGB LED: %s", esp_err_to_name(err));
    status_led_cleanup();
    return err;
  }

  err = status_led_create_activity_timer();

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create activity timer: %s", esp_err_to_name(err));
    status_led_cleanup();
    return err;
  }

  err = status_led_apply_rgb(STATUS_LED_INITIALIZING);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set initial RGB state: %s", esp_err_to_name(err));
    status_led_cleanup();
    return err;
  }

  s_status_led.state = STATUS_LED_INITIALIZING;
  s_status_led.initialized = true;

  ESP_LOGI(TAG, "Initialized RGB GPIO %d and activity GPIO %d",
           CONFIG_APP_STATUS_RGB_LED_GPIO, CONFIG_APP_STATUS_ACTIVITY_LED_GPIO);

  return ESP_OK;
}

esp_err_t status_led_set_state(status_led_state_t state) {
  if (!s_status_led.initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t err = status_led_apply_rgb(state);

  if (err != ESP_OK) {
    return err;
  }

  s_status_led.state = state;
  return ESP_OK;
}

esp_err_t status_led_activity_pulse(void) {
  if (!s_status_led.initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t err = gpio_set_level(CONFIG_APP_STATUS_ACTIVITY_LED_GPIO, 1);

  if (err != ESP_OK) {
    return err;
  }

  if (esp_timer_is_active(s_status_led.activity_timer)) {
    err = esp_timer_stop(s_status_led.activity_timer);

    if (err != ESP_OK) {
      (void)gpio_set_level(CONFIG_APP_STATUS_ACTIVITY_LED_GPIO, 0);
      return err;
    }
  }

  err = esp_timer_start_once(s_status_led.activity_timer,
                             ACTIVITY_PULSE_DURATION_US);

  if (err != ESP_OK) {
    (void)gpio_set_level(CONFIG_APP_STATUS_ACTIVITY_LED_GPIO, 0);
    return err;
  }

  return ESP_OK;
}
