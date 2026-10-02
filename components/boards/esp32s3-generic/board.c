/**
 * @file board.c
 * @brief ESP32-S3 Generic board implementation
 *
 * Minimal implementation for generic ESP32-S3 dev boards with external I2S DAC.
 * No board-specific initialization required.
 */

#include "iot_board.h"

#include "driver/gpio.h"
#include "esp_log.h"

#if defined(CONFIG_DISPLAY_ENABLED) && defined(CONFIG_DISPLAY_BUS_I2C)
#include "driver/i2c_master.h"
#define BOARD_HAS_DISPLAY_I2C 1
#endif

static const char TAG[] = "ESP32S3-Generic";

static bool s_board_initialized = false;

#ifdef BOARD_HAS_DISPLAY_I2C
static i2c_master_bus_handle_t s_i2c_disp_bus_handle = NULL;

static esp_err_t init_display_i2c_bus(void) {
  i2c_master_bus_config_t i2c_cfg = {
      .i2c_port = -1,
      .sda_io_num = CONFIG_DISPLAY_I2C_SDA,
      .scl_io_num = CONFIG_DISPLAY_I2C_SCL,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  esp_err_t err = i2c_new_master_bus(&i2c_cfg, &s_i2c_disp_bus_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize display I2C bus: %s",
             esp_err_to_name(err));
    return err;
  }
  ESP_LOGI(TAG, "Display I2C bus initialized: sda=%d, scl=%d",
           CONFIG_DISPLAY_I2C_SDA, CONFIG_DISPLAY_I2C_SCL);
  return ESP_OK;
}
#endif

#ifdef CONFIG_MUTE_GPIO
static esp_err_t init_mute_gpio(void) {
  if (CONFIG_MUTE_GPIO < 0) {
    return ESP_OK;
  }

  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << CONFIG_MUTE_GPIO),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t err = gpio_config(&io_conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure mute GPIO: %s", esp_err_to_name(err));
    return err;
  }

  // Initialize to unmuted state — set opposite of active level
  gpio_set_level(CONFIG_MUTE_GPIO, !CONFIG_MUTE_GPIO_LEVEL);

  ESP_LOGI(TAG, "Mute GPIO %d initialized (active %s, init %s)",
           CONFIG_MUTE_GPIO, CONFIG_MUTE_GPIO_LEVEL ? "high" : "low",
           CONFIG_MUTE_GPIO_LEVEL ? "low" : "high");
  return ESP_OK;
}
#endif

const char *iot_board_get_info(void) {
  return BOARD_NAME;
}

bool iot_board_is_init(void) {
  return s_board_initialized;
}

board_res_handle_t iot_board_get_handle(int id) {
#ifdef BOARD_HAS_DISPLAY_I2C
  if (id == BOARD_I2C_DISP_ID) {
    return (board_res_handle_t)s_i2c_disp_bus_handle;
  }
#endif
  (void)id;
  return NULL;
}

esp_err_t iot_board_init(void) {
  if (s_board_initialized) {
    ESP_LOGW(TAG, "Board already initialized");
    return ESP_OK;
  }

#ifdef CONFIG_MUTE_GPIO
  esp_err_t err = init_mute_gpio();
  if (err != ESP_OK) {
    return err;
  }
#endif

#ifdef BOARD_HAS_DISPLAY_I2C
  esp_err_t i2c_err = init_display_i2c_bus();
  if (i2c_err != ESP_OK) {
    return i2c_err;
  }
#endif

  s_board_initialized = true;
  ESP_LOGI(TAG, "Generic board initialized (no board-specific init needed)");
  return ESP_OK;
}

esp_err_t iot_board_deinit(void) {
  s_board_initialized = false;
  return ESP_OK;
}
