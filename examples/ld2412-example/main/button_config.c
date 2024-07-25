#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "iot_button.h"
#include "esp_sleep.h"
#include "esp_idf_version.h"

#include "button_config.h"

static const char *TAG = "iot_button";

const char *button_event_table[] = {
    "BUTTON_PRESS_DOWN",
    "BUTTON_PRESS_UP",
    "BUTTON_PRESS_REPEAT",
    "BUTTON_PRESS_REPEAT_DONE",
    "BUTTON_SINGLE_CLICK",
    "BUTTON_DOUBLE_CLICK",
    "BUTTON_MULTIPLE_CLICK",
    "BUTTON_LONG_PRESS_START",
    "BUTTON_LONG_PRESS_HOLD",
    "BUTTON_LONG_PRESS_UP",
};

static void button_event_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Button event %s", button_event_table[(button_event_t)data]);
}

static void button_single_click_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Button event %s", button_event_table[(button_event_t)data]);
    // read_firmware_version();

    // Command value (2 bytes) baud rate selection index 0x0004 for 57600, 0x0005 for 115200
    uint8_t cmd_val[2] = {0x04, 0x00};
    set_baud_rate(cmd_val);
}

static void button_double_click_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Button event %s", button_event_table[(button_event_t)data]);
    control_engineering_mode();
}

void button_init(uint32_t button_num)
{
  button_config_t btn_cfg = {
    .type = BUTTON_TYPE_GPIO,
    .gpio_button_config = {
        .gpio_num = button_num,
        .active_level = BUTTON_ACTIVE_LEVEL,
      },
  };
  button_handle_t btn = iot_button_create(&btn_cfg);
  assert(btn);
  esp_err_t err = iot_button_register_cb(btn, BUTTON_PRESS_DOWN, button_event_cb, (void *)BUTTON_PRESS_DOWN);
  err |= iot_button_register_cb(btn, BUTTON_PRESS_UP, button_event_cb, (void *)BUTTON_PRESS_UP);
  err |= iot_button_register_cb(btn, BUTTON_PRESS_REPEAT, button_event_cb, (void *)BUTTON_PRESS_REPEAT);
  err |= iot_button_register_cb(btn, BUTTON_PRESS_REPEAT_DONE, button_event_cb, (void *)BUTTON_PRESS_REPEAT_DONE);
  err |= iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, button_single_click_cb, (void *)BUTTON_SINGLE_CLICK);
  err |= iot_button_register_cb(btn, BUTTON_DOUBLE_CLICK, button_double_click_cb, (void *)BUTTON_DOUBLE_CLICK);
  err |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, button_event_cb, (void *)BUTTON_LONG_PRESS_START);
  err |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_HOLD, button_event_cb, (void *)BUTTON_LONG_PRESS_HOLD);
  err |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_UP, button_event_cb, (void *)BUTTON_LONG_PRESS_UP);
  ESP_ERROR_CHECK(err);
}

// void power_save_init(void)
// {
//     esp_pm_config_t pm_config = {
//         .max_freq_mhz = CONFIG_EXAMPLE_MAX_CPU_FREQ_MHZ,
//         .min_freq_mhz = CONFIG_EXAMPLE_MIN_CPU_FREQ_MHZ,
// #if CONFIG_FREERTOS_USE_TICKLESS_IDLE
//         .light_sleep_enable = true
// #endif
//     };
//     ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
// }