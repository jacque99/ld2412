#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include <esp_timer.h>

#include "ld2412.h"

static const char *TAG = "LD2412DEMO";

void app_main(void)
{
  // uint8_t fw_version[6];

  uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_driver_install(RADAR_UART_PORT_NUMBER, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(RADAR_UART_PORT_NUMBER, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(RADAR_UART_PORT_NUMBER, 5, 4, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  ld2412_config_t config = {
    .uart_port = RADAR_UART_PORT_NUMBER,
    .last_ts = esp_timer_get_time(),
  };
  ld2412_handle_t handle = NULL;
  ESP_ERROR_CHECK(ld2412_init(&config, &handle));

  ESP_LOGE(TAG, "Initialization complete:");
  xTaskCreate(receive_task, "uart_rx_task", 1024*2, NULL, 4, NULL);

  // memset(fw_version, 0, sizeof(fw_version));
  
  // if (!(read_firmware_version(handle, fw_version))) {
  //   ESP_LOGE(TAG, "Error getting FW Version message:");

  // }

  // // Radar ACK (success)
  // // Command word: 2 bytes For example 0x01A0
  // // Return value: 2-bytes ACK status (0 successful, 1 failed) + 2-bytes firmware type (0x2412)+2-bytes major
  // //                version number+4-bytes minor version number
  // ESP_LOGI(TAG, "LD2412 firmware type: %02X.%02X", fw_version[5], fw_version[4]);
  // ESP_LOGI(TAG, "LD2412 firmware version: V%02X.%02X.%02X.%02X.%02X.%02X", fw_version[7], fw_version[6], fw_version[11], fw_version[10], fw_version[9], fw_version[8]);
  
}