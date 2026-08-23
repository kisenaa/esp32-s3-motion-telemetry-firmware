#include "mma8451q.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <stddef.h>

/* Config used*/
/*
  - CONFIG_APP_MMA8451Q_I2C_PORT
  - CONFIG_APP_MMA8451Q_SDA_GPIO
  - CONFIG_APP_MMA8451Q_SCL_GPIO
  - CONFIG_APP_MMA8451Q_INT1_GPIO
  - CONFIG_APP_MMA8451Q_I2C_ADDRESS
  - CONFIG_APP_MMA8451Q_I2C_FREQUENCY_HZ
  - CONFIG_APP_MMA8451Q_I2C_TIMEOUT_MS
  - CONFIG_APP_MMA8451Q_DEFAULT_RANGE
  - CONFIG_APP_MMA8451Q_DEFAULT_ODR
*/

static const char *TAG = "mma8451q";

static constexpr uint8_t REG_OUT_X_MSB = 0x01;
static constexpr uint8_t REG_INT_SOURCE = 0x0C;
static constexpr uint8_t REG_WHO_AM_I = 0x0D;
static constexpr uint8_t REG_XYZ_DATA_CFG = 0x0E;
static constexpr uint8_t REG_PL_STATUS = 0x10;
static constexpr uint8_t REG_PL_CFG = 0x11;
static constexpr uint8_t REG_CTRL1 = 0x2A;
static constexpr uint8_t REG_CTRL3 = 0x2C;
static constexpr uint8_t REG_CTRL4 = 0x2D;
static constexpr uint8_t REG_CTRL5 = 0x2E;

static constexpr uint8_t WHO_AM_I_VALUE = 0x1A;

static constexpr uint8_t CTRL1_ACTIVE_MASK = (1U << 0);
static constexpr uint8_t CTRL1_ODR_MASK = (7U << 3);
static constexpr uint8_t CTRL1_ODR_SHIFT = 3;

static constexpr uint8_t XYZ_RANGE_MASK = 0x03U;

static constexpr uint8_t INTERRUPT_DATA_READY_MASK = (1U << 0);
static constexpr uint8_t CTRL3_PUSH_PULL_OPEN_DRAIN_MASK = (1U << 0);
static constexpr uint8_t CTRL3_INTERRUPT_POLARITY_MASK = (1U << 1);

static constexpr uint8_t PL_ENABLE_MASK = (1U << 6);
static constexpr uint8_t PL_CHANGED_MASK = (1U << 7);
static constexpr uint8_t PL_LOCKOUT_MASK = (1U << 6);
static constexpr uint8_t PL_ORIENTATION_MASK = 0x06U;
static constexpr uint8_t PL_ORIENTATION_SHIFT = 1;
static constexpr uint8_t PL_BACK_FRONT_MASK = (1U << 0);

static constexpr size_t ACCELERATION_BYTE_COUNT = 6;

static constexpr float COUNTS_PER_G_2G = 4096.0f;
static constexpr float COUNTS_PER_G_4G = 2048.0f;
static constexpr float COUNTS_PER_G_8G = 1024.0f;

static i2c_master_bus_handle_t s_bus = nullptr;
static i2c_master_dev_handle_t s_device = nullptr;

static mma8451q_range_t s_range = MMA8451Q_RANGE_2G;
static bool s_initialized = false;

static esp_err_t mma8451q_read_registers(uint8_t register_address,
                                         uint8_t *data, size_t length) {
  return i2c_master_transmit_receive(s_device, &register_address,
                                     sizeof(register_address), data, length,
                                     CONFIG_APP_MMA8451Q_I2C_TIMEOUT_MS);
}

static esp_err_t mma8451q_read_register(uint8_t register_address,
                                        uint8_t *value) {
  return mma8451q_read_registers(register_address, value, sizeof(*value));
}

static esp_err_t mma8451q_write_register(uint8_t register_address,
                                         uint8_t value) {
  const uint8_t transaction[] = {
      register_address,
      value,
  };

  return i2c_master_transmit(s_device, transaction, sizeof(transaction),
                             CONFIG_APP_MMA8451Q_I2C_TIMEOUT_MS);
}

static bool mma8451q_range_is_valid(mma8451q_range_t range) {
  return range >= MMA8451Q_RANGE_2G && range <= MMA8451Q_RANGE_8G;
}

static bool mma8451q_odr_is_valid(mma8451q_odr_t odr) {
  return odr >= MMA8451Q_ODR_800_HZ && odr <= MMA8451Q_ODR_1_56_HZ;
}

static float mma8451q_counts_per_g(mma8451q_range_t range) {
  switch (range) {
  case MMA8451Q_RANGE_2G:
    return COUNTS_PER_G_2G;

  case MMA8451Q_RANGE_4G:
    return COUNTS_PER_G_4G;

  case MMA8451Q_RANGE_8G:
    return COUNTS_PER_G_8G;

  default:
    return COUNTS_PER_G_2G;
  }
}

static int16_t mma8451q_decode_14bit(uint8_t msb, uint8_t lsb) {
  uint16_t raw = (uint16_t)((uint16_t)msb << 8) | (uint16_t)lsb;

  raw >>= 2;

  if ((raw & 0x2000U) != 0U) {
    raw |= 0xC000U;
  }

  return (int16_t)raw;
}

static esp_err_t mma8451q_enter_standby(bool *was_active) {
  if (was_active == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t ctrl1;
  esp_err_t err = mma8451q_read_register(REG_CTRL1, &ctrl1);

  if (err != ESP_OK) {
    return err;
  }

  *was_active = (ctrl1 & CTRL1_ACTIVE_MASK) != 0U;

  if (!*was_active) {
    return ESP_OK;
  }

  ctrl1 &= (uint8_t)~CTRL1_ACTIVE_MASK;

  return mma8451q_write_register(REG_CTRL1, ctrl1);
}

static esp_err_t mma8451q_restore_active(bool was_active) {
  if (!was_active) {
    return ESP_OK;
  }

  uint8_t ctrl1;
  esp_err_t err = mma8451q_read_register(REG_CTRL1, &ctrl1);

  if (err != ESP_OK) {
    return err;
  }

  ctrl1 |= CTRL1_ACTIVE_MASK;

  return mma8451q_write_register(REG_CTRL1, ctrl1);
}

static void mma8451q_cleanup_initialization(void) {
  if (s_device != nullptr) {
    const esp_err_t err = i2c_master_bus_rm_device(s_device);

    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to remove I2C device during cleanup: %s",
               esp_err_to_name(err));
    }

    s_device = nullptr;
  }

  if (s_bus != nullptr) {
    const esp_err_t err = i2c_del_master_bus(s_bus);

    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to delete I2C bus during cleanup: %s",
               esp_err_to_name(err));
    }

    s_bus = nullptr;
  }
}

static esp_err_t mma8451q_configure_range_in_standby(mma8451q_range_t range) {
  if (!mma8451q_range_is_valid(range)) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t data_config;
  esp_err_t err = mma8451q_read_register(REG_XYZ_DATA_CFG, &data_config);

  if (err != ESP_OK) {
    return err;
  }

  data_config &= (uint8_t)~XYZ_RANGE_MASK;
  data_config |= (uint8_t)range;

  err = mma8451q_write_register(REG_XYZ_DATA_CFG, data_config);

  if (err == ESP_OK) {
    s_range = range;
  }

  return err;
}

static esp_err_t mma8451q_configure_odr_in_standby(mma8451q_odr_t odr) {
  if (!mma8451q_odr_is_valid(odr)) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t ctrl1;
  esp_err_t err = mma8451q_read_register(REG_CTRL1, &ctrl1);

  if (err != ESP_OK) {
    return err;
  }

  ctrl1 &= (uint8_t)~CTRL1_ODR_MASK;
  ctrl1 |= (uint8_t)odr << CTRL1_ODR_SHIFT;
  ctrl1 &= (uint8_t)~CTRL1_ACTIVE_MASK;

  return mma8451q_write_register(REG_CTRL1, ctrl1);
}

esp_err_t mma8451q_init(void) {
  if (s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  const i2c_master_bus_config_t bus_config = {
      .i2c_port = CONFIG_APP_MMA8451Q_I2C_PORT,
      .sda_io_num = CONFIG_APP_MMA8451Q_SDA_GPIO,
      .scl_io_num = CONFIG_APP_MMA8451Q_SCL_GPIO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
    return err;
  }

  err = i2c_master_probe(s_bus, CONFIG_APP_MMA8451Q_I2C_ADDRESS,
                         CONFIG_APP_MMA8451Q_I2C_TIMEOUT_MS);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "MMA8451Q not found at address 0x%02X: %s",
             CONFIG_APP_MMA8451Q_I2C_ADDRESS, esp_err_to_name(err));
    mma8451q_cleanup_initialization();
    return ESP_ERR_NOT_FOUND;
  }

  const i2c_device_config_t device_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = CONFIG_APP_MMA8451Q_I2C_ADDRESS,
      .scl_speed_hz = CONFIG_APP_MMA8451Q_I2C_FREQUENCY_HZ,
  };

  err = i2c_master_bus_add_device(s_bus, &device_config, &s_device);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add MMA8451Q I2C device: %s",
             esp_err_to_name(err));
    mma8451q_cleanup_initialization();
    return err;
  }

  uint8_t device_id;
  err = mma8451q_read_register(REG_WHO_AM_I, &device_id);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(err));
    mma8451q_cleanup_initialization();
    return err;
  }

  if (device_id != WHO_AM_I_VALUE) {
    ESP_LOGE(TAG, "Unexpected WHO_AM_I value: 0x%02X", device_id);
    mma8451q_cleanup_initialization();
    return ESP_ERR_NOT_FOUND;
  }

  ESP_LOGI(TAG, "Detected MMA8451Q at address 0x%02X",
           CONFIG_APP_MMA8451Q_I2C_ADDRESS);

  uint8_t ctrl1;
  err = mma8451q_read_register(REG_CTRL1, &ctrl1);

  if (err != ESP_OK) {
    mma8451q_cleanup_initialization();
    return err;
  }

  ctrl1 &= (uint8_t)~CTRL1_ACTIVE_MASK;
  err = mma8451q_write_register(REG_CTRL1, ctrl1);

  if (err != ESP_OK) {
    mma8451q_cleanup_initialization();
    return err;
  }

  err = mma8451q_configure_range_in_standby(
      (mma8451q_range_t)CONFIG_APP_MMA8451Q_DEFAULT_RANGE);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure measurement range: %s",
             esp_err_to_name(err));
    mma8451q_cleanup_initialization();
    return err;
  }

  err = mma8451q_configure_odr_in_standby(
      (mma8451q_odr_t)CONFIG_APP_MMA8451Q_DEFAULT_ODR);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure output data rate: %s",
             esp_err_to_name(err));
    mma8451q_cleanup_initialization();
    return err;
  }

  err = mma8451q_read_register(REG_CTRL1, &ctrl1);

  if (err == ESP_OK) {
    err = mma8451q_write_register(REG_CTRL1, ctrl1 | CTRL1_ACTIVE_MASK);
  }

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to activate MMA8451Q: %s", esp_err_to_name(err));
    mma8451q_cleanup_initialization();
    return err;
  }

  s_initialized = true;

  ESP_LOGI(TAG, "MMA8451Q initialized: range index %d, ODR index %d",
           CONFIG_APP_MMA8451Q_DEFAULT_RANGE, CONFIG_APP_MMA8451Q_DEFAULT_ODR);

  return ESP_OK;
}

esp_err_t mma8451q_deinit(void) {
  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t result = ESP_OK;

  uint8_t ctrl1;
  esp_err_t err = mma8451q_read_register(REG_CTRL1, &ctrl1);

  if (err == ESP_OK) {
    err =
        mma8451q_write_register(REG_CTRL1, ctrl1 & (uint8_t)~CTRL1_ACTIVE_MASK);
  }

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to place sensor in standby: %s",
             esp_err_to_name(err));
    result = err;
  }

  err = i2c_master_bus_rm_device(s_device);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to remove MMA8451Q device: %s", esp_err_to_name(err));
    result = err;
  } else {
    s_device = nullptr;
  }

  if (s_device == nullptr) {
    err = i2c_del_master_bus(s_bus);

    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to delete I2C bus: %s", esp_err_to_name(err));
      result = err;
    } else {
      s_bus = nullptr;
    }
  }

  if (s_device == nullptr && s_bus == nullptr) {
    s_initialized = false;
    ESP_LOGI(TAG, "MMA8451Q deinitialized");
  }

  return result;
}

esp_err_t mma8451q_set_range(mma8451q_range_t range) {
  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  if (!mma8451q_range_is_valid(range)) {
    return ESP_ERR_INVALID_ARG;
  }

  bool was_active;
  esp_err_t err = mma8451q_enter_standby(&was_active);

  if (err != ESP_OK) {
    return err;
  }

  err = mma8451q_configure_range_in_standby(range);

  const esp_err_t restore_err = mma8451q_restore_active(was_active);

  if (err != ESP_OK) {
    return err;
  }

  if (restore_err != ESP_OK) {
    ESP_LOGE(TAG, "Range changed but active mode restoration failed: %s",
             esp_err_to_name(restore_err));
    return restore_err;
  }

  ESP_LOGI(TAG, "Measurement range changed to +/-%d g",
           range == MMA8451Q_RANGE_2G   ? 2
           : range == MMA8451Q_RANGE_4G ? 4
                                        : 8);

  return ESP_OK;
}

esp_err_t mma8451q_read_sample(mma8451q_sample_t *sample) {
  if (sample == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  uint8_t data[ACCELERATION_BYTE_COUNT];

  esp_err_t err = mma8451q_read_registers(REG_OUT_X_MSB, data, sizeof(data));

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read acceleration data: %s", esp_err_to_name(err));
    return err;
  }

  sample->x_raw = mma8451q_decode_14bit(data[0], data[1]);
  sample->y_raw = mma8451q_decode_14bit(data[2], data[3]);
  sample->z_raw = mma8451q_decode_14bit(data[4], data[5]);

  const float counts_per_g = mma8451q_counts_per_g(s_range);

  sample->x_g = (float)sample->x_raw / counts_per_g;
  sample->y_g = (float)sample->y_raw / counts_per_g;
  sample->z_g = (float)sample->z_raw / counts_per_g;

  return ESP_OK;
}

esp_err_t mma8451q_set_odr(mma8451q_odr_t odr) {
  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  if (!mma8451q_odr_is_valid(odr)) {
    return ESP_ERR_INVALID_ARG;
  }

  bool was_active;
  esp_err_t err = mma8451q_enter_standby(&was_active);

  if (err != ESP_OK) {
    return err;
  }

  err = mma8451q_configure_odr_in_standby(odr);

  const esp_err_t restore_err = mma8451q_restore_active(was_active);

  if (err != ESP_OK) {
    return err;
  }

  if (restore_err != ESP_OK) {
    ESP_LOGE(TAG, "ODR changed but active mode restoration failed: %s",
             esp_err_to_name(restore_err));
    return restore_err;
  }

  ESP_LOGI(TAG, "Output data rate changed to index %d", (int)odr);

  return ESP_OK;
}

static esp_err_t mma8451q_configure_interrupt_output_in_standby(void) {
  uint8_t ctrl3;
  esp_err_t err = mma8451q_read_register(REG_CTRL3, &ctrl3);

  if (err != ESP_OK) {
    return err;
  }

  ctrl3 &= (uint8_t)~CTRL3_PUSH_PULL_OPEN_DRAIN_MASK;
  ctrl3 &= (uint8_t)~CTRL3_INTERRUPT_POLARITY_MASK;

  return mma8451q_write_register(REG_CTRL3, ctrl3);
}

esp_err_t mma8451q_enable_data_ready_interrupt(bool enable) {
  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  bool was_active;
  esp_err_t err = mma8451q_enter_standby(&was_active);

  if (err != ESP_OK) {
    return err;
  }

  if (enable) {
    err = mma8451q_configure_interrupt_output_in_standby();
  }

  uint8_t ctrl4;
  uint8_t ctrl5;

  if (err == ESP_OK) {
    err = mma8451q_read_register(REG_CTRL4, &ctrl4);
  }

  if (err == ESP_OK) {
    err = mma8451q_read_register(REG_CTRL5, &ctrl5);
  }

  if (err == ESP_OK && enable) {
    ctrl4 |= INTERRUPT_DATA_READY_MASK;
    ctrl5 |= INTERRUPT_DATA_READY_MASK;
  } else if (err == ESP_OK) {
    ctrl4 &= (uint8_t)~INTERRUPT_DATA_READY_MASK;
    ctrl5 &= (uint8_t)~INTERRUPT_DATA_READY_MASK;
  }

  if (err == ESP_OK) {
    err = mma8451q_write_register(REG_CTRL5, ctrl5);
  }

  if (err == ESP_OK) {
    err = mma8451q_write_register(REG_CTRL4, ctrl4);
  }

  const esp_err_t restore_err = mma8451q_restore_active(was_active);

  if (err != ESP_OK) {
    return err;
  }

  if (restore_err != ESP_OK) {
    return restore_err;
  }

  ESP_LOGI(TAG, "Data-ready interrupt %s on active-low INT1",
           enable ? "enabled" : "disabled");

  return ESP_OK;
}

esp_err_t mma8451q_read_interrupt_source(uint8_t *sources) {
  if (sources == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  return mma8451q_read_register(REG_INT_SOURCE, sources);
}

esp_err_t mma8451q_enable_orientation(bool enable) {
  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  bool was_active;
  esp_err_t err = mma8451q_enter_standby(&was_active);

  if (err != ESP_OK) {
    return err;
  }

  uint8_t pl_config;
  err = mma8451q_read_register(REG_PL_CFG, &pl_config);

  if (err == ESP_OK) {
    if (enable) {
      pl_config |= PL_ENABLE_MASK;
    } else {
      pl_config &= (uint8_t)~PL_ENABLE_MASK;
    }

    err = mma8451q_write_register(REG_PL_CFG, pl_config);
  }

  const esp_err_t restore_err = mma8451q_restore_active(was_active);

  if (err != ESP_OK) {
    return err;
  }

  if (restore_err != ESP_OK) {
    return restore_err;
  }

  ESP_LOGI(TAG, "Orientation detection %s", enable ? "enabled" : "disabled");

  return ESP_OK;
}

esp_err_t mma8451q_read_orientation(mma8451q_orientation_status_t *status) {
  if (status == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  uint8_t pl_status;
  esp_err_t err = mma8451q_read_register(REG_PL_STATUS, &pl_status);

  if (err != ESP_OK) {
    return err;
  }

  status->changed = (pl_status & PL_CHANGED_MASK) != 0U;
  status->lockout = (pl_status & PL_LOCKOUT_MASK) != 0U;
  status->back_facing = (pl_status & PL_BACK_FRONT_MASK) != 0U;

  status->orientation =
      (mma8451q_orientation_t)((pl_status & PL_ORIENTATION_MASK) >>
                               PL_ORIENTATION_SHIFT);

  return ESP_OK;
}
