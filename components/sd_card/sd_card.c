#include "sd_card.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdkconfig.h"
#include "sdmmc_cmd.h"
#include <dirent.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* Config Used*/
/*
  - CONFIG_APP_SD_SPI_HOST
  - CONFIG_APP_SD_SPI_FREQUENCY_KHZ
  - CONFIG_APP_SD_MOSI_GPIO
  - CONFIG_APP_SD_MISO_GPIO
  - CONFIG_APP_SD_SCLK_GPIO
  - CONFIG_APP_SD_CS_GPIO
  - CONFIG_APP_SD_MOUNT_POINT
  - CONFIG_APP_SD_CSV_FILENAME
  - CONFIG_APP_SD_MAX_OPEN_FILES
  - CONFIG_APP_SD_ALLOCATION_UNIT_SIZE
  - CONFIG_APP_SD_FORMAT_IF_MOUNT_FAILED
*/

static const char *TAG = "sd_card";

static constexpr char CSV_HEADER[] =
    "timestamp_us,x_raw,y_raw,z_raw,x_g,y_g,z_g\n";

static constexpr unsigned int MAX_FILE_INDEX = 9999;
static constexpr size_t CSV_PATH_SIZE = 128;
static constexpr int SPI_UNUSED_GPIO = -1;
static constexpr int SD_MAX_TRANSFER_SIZE = 4096;

static sdmmc_card_t *s_card = nullptr;
static FILE *s_csv_file = nullptr;

static spi_host_device_t s_spi_host = (spi_host_device_t)CONFIG_APP_SD_SPI_HOST;

static bool s_spi_bus_initialized = false;
static bool s_initialized = false;

static char s_csv_path[CSV_PATH_SIZE];

static esp_err_t sd_card_build_next_csv_path(void) {
  DIR *dir = opendir(CONFIG_APP_SD_MOUNT_POINT);

  if (dir == nullptr) {
    ESP_LOGE(TAG, "Failed to open %s", CONFIG_APP_SD_MOUNT_POINT);
    return ESP_FAIL;
  }

  unsigned int max_index = 0;

  const char *prefix = CONFIG_APP_SD_CSV_FILENAME;
  const size_t prefix_len = strlen(prefix);

  struct dirent *entry;

  while ((entry = readdir(dir)) != nullptr) {
    const char *name = entry->d_name;

    if (strncmp(name, prefix, prefix_len) != 0) {
      continue;
    }

    if (name[prefix_len] != '_') {
      continue;
    }

    unsigned int index = 0;
    int consumed = 0;

    if (sscanf(name + prefix_len + 1, "%4u.csv%n", &index, &consumed) != 1) {
      continue;
    }

    const char *suffix = name + prefix_len + 1;

    if (suffix[consumed] == '\0' && index > max_index) {
      max_index = index;
    }
  }

  closedir(dir);

  if (max_index >= MAX_FILE_INDEX) {
    ESP_LOGE(TAG, "Maximum telemetry file index reached");
    return ESP_ERR_INVALID_SIZE;
  }

  const unsigned int next_index = max_index + 1;

  const int written = snprintf(s_csv_path, sizeof(s_csv_path), "%s/%s_%04u.csv",
                               CONFIG_APP_SD_MOUNT_POINT,
                               CONFIG_APP_SD_CSV_FILENAME, next_index);

  if (written < 0 || (size_t)written >= sizeof(s_csv_path)) {
    ESP_LOGE(TAG, "CSV path is too long");
    return ESP_ERR_INVALID_SIZE;
  }

  return ESP_OK;
}

static esp_err_t sd_card_mount(void) {
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = (int)s_spi_host;
  host.max_freq_khz = CONFIG_APP_SD_SPI_FREQUENCY_KHZ;

  const spi_bus_config_t bus_config = {
      .mosi_io_num = CONFIG_APP_SD_MOSI_GPIO,
      .miso_io_num = CONFIG_APP_SD_MISO_GPIO,
      .sclk_io_num = CONFIG_APP_SD_SCLK_GPIO,
      .quadwp_io_num = SPI_UNUSED_GPIO,
      .quadhd_io_num = SPI_UNUSED_GPIO,
      .max_transfer_sz = SD_MAX_TRANSFER_SIZE,
  };

  esp_err_t err = spi_bus_initialize(s_spi_host, &bus_config, SPI_DMA_CH_AUTO);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(err));
    return err;
  }

  s_spi_bus_initialized = true;

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.host_id = s_spi_host;
  slot_config.gpio_cs = CONFIG_APP_SD_CS_GPIO;

  const esp_vfs_fat_mount_config_t mount_config = {
#ifdef CONFIG_APP_SD_FORMAT_IF_MOUNT_FAILED
      .format_if_mount_failed = true,
#else
      .format_if_mount_failed = false,
#endif
      .max_files = CONFIG_APP_SD_MAX_OPEN_FILES,
      .allocation_unit_size = CONFIG_APP_SD_ALLOCATION_UNIT_SIZE,
  };

  err = esp_vfs_fat_sdspi_mount(CONFIG_APP_SD_MOUNT_POINT, &host, &slot_config,
                                &mount_config, &s_card);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(err));

    const esp_err_t free_err = spi_bus_free(s_spi_host);

    if (free_err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to release SPI bus after mount failure: %s",
               esp_err_to_name(free_err));
    }

    s_spi_bus_initialized = false;
    return err;
  }

  sdmmc_card_print_info(stdout, s_card);

  return ESP_OK;
}

esp_err_t sd_card_init(void) {
  if (s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  const esp_err_t err = sd_card_mount();

  if (err != ESP_OK) {
    return err;
  }

  s_initialized = true;

  ESP_LOGI(TAG, "SD card mounted at %s", CONFIG_APP_SD_MOUNT_POINT);
  ESP_LOGI(TAG, "Card name: %s", s_card->cid.name);

  return ESP_OK;
}

esp_err_t sd_card_start(void) {
  if (!s_initialized || s_csv_file != nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  const esp_err_t err = sd_card_build_next_csv_path();

  if (err != ESP_OK) {
    return err;
  }

  s_csv_file = fopen(s_csv_path, "w");

  if (s_csv_file == nullptr) {
    ESP_LOGE(TAG, "Failed to open %s", s_csv_path);
    return ESP_FAIL;
  }

  if (fputs(CSV_HEADER, s_csv_file) == EOF || fflush(s_csv_file) != 0) {
    ESP_LOGE(TAG, "Failed to write the CSV header");
    (void)fclose(s_csv_file);
    s_csv_file = nullptr;
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "CSV logging started: %s", s_csv_path);
  return ESP_OK;
}

esp_err_t sd_card_append(const sd_card_record_t *record) {
  if (record == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  if (s_csv_file == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  const int written = fprintf(
      s_csv_file,
      "%" PRId64 ",%" PRId16 ",%" PRId16 ",%" PRId16 ",%.6f,%.6f,%.6f\n",
      record->timestamp_us, record->x_raw, record->y_raw, record->z_raw,
      (double)record->x_g, (double)record->y_g, (double)record->z_g);

  if (written < 0) {
    ESP_LOGE(TAG, "Failed to append telemetry record");
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t sd_card_flush(void) {
  if (s_csv_file == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  if (fflush(s_csv_file) != 0) {
    ESP_LOGE(TAG, "Failed to flush telemetry data");
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t sd_card_stop(void) {
  if (s_csv_file == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t result = sd_card_flush();

  if (fclose(s_csv_file) != 0) {
    ESP_LOGE(TAG, "Failed to close telemetry CSV file");
    result = ESP_FAIL;
  }

  s_csv_file = nullptr;

  if (result == ESP_OK) {
    ESP_LOGI(TAG, "CSV logging stopped");
  }

  return result;
}

void sd_card_deinit(void) {
  if (!s_initialized) {
    return;
  }

  if (s_csv_file != nullptr) {
    const esp_err_t stop_err = sd_card_stop();

    if (stop_err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to stop logging during deinitialization: %s",
               esp_err_to_name(stop_err));
    }
  }

  const esp_err_t unmount_err =
      esp_vfs_fat_sdcard_unmount(CONFIG_APP_SD_MOUNT_POINT, s_card);

  if (unmount_err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to unmount SD card: %s",
             esp_err_to_name(unmount_err));
  } else {
    ESP_LOGI(TAG, "SD card unmounted");
  }

  s_card = nullptr;

  if (s_spi_bus_initialized) {
    const esp_err_t free_err = spi_bus_free(s_spi_host);

    if (free_err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to release SPI bus: %s", esp_err_to_name(free_err));
    }

    s_spi_bus_initialized = false;
  }

  s_initialized = false;
}
