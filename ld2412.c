#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include <esp_timer.h>

#include "uart_config.h"
#include "ld2412.h"

static const char *TAG = "LD2412";

static bool current_engineering_mode = false;

void send_command(uint8_t *command_str, uint8_t *command_val, int command_val_len) {
  // Initialize frame length Frame header 4 bytes + intra frame length 2 bytes
  int frame_data_index = 0;          
  int intra_frame_data_length = 0;          
  
  // Frame header bytes
  hUart.uart_txBuffer[0] = 0xFD;
  hUart.uart_txBuffer[1] = 0xFC;
  hUart.uart_txBuffer[2] = 0xFB;
  hUart.uart_txBuffer[3] = 0xFA;
  frame_data_index += 4;

  // intra frame data length bytes
  hUart.uart_txBuffer[4] = 0;
  hUart.uart_txBuffer[5] = 0;
  frame_data_index += 2;
  // command word bytes
  hUart.uart_txBuffer[6] = command_str[0];
  hUart.uart_txBuffer[7] = command_str[1];
  frame_data_index += 2;
  intra_frame_data_length += 2;
  // command value bytes
  if (command_val[0] != '\0') {
    for (int i = 0; i < command_val_len; i++)
    {
      hUart.uart_txBuffer[frame_data_index+i] = command_val[i];
      frame_data_index++;
      intra_frame_data_length++;
    }
  }
  // intra frame data length
  hUart.uart_txBuffer[4] = (uint8_t)(intra_frame_data_length & 0xFF);
  hUart.uart_txBuffer[5] = (uint8_t)((intra_frame_data_length >> 8) & 0xFF);

  hUart.uart_txBuffer[frame_data_index] = 0x04;
  hUart.uart_txBuffer[frame_data_index+1] = 0x03;
  hUart.uart_txBuffer[frame_data_index+2] = 0x02;
  hUart.uart_txBuffer[frame_data_index+3] = 0x01;
  frame_data_index += 4;

  hUart.uart_txPacketSize = frame_data_index;
    xQueueSendToBack(uartTx_queue, &hUart, portMAX_DELAY);
}

void control_config_mode(bool enable) {
  // Command word (2 bytes) 0x00FF or 0x00FE
  uint8_t cmd[2] = {enable ? 0xFF : 0xFE, 0x00};
  uint8_t cmd_val[2] = {};
  int cmd_val_len = enable ? 2 : 0;
  if (enable) {
    // Command value (2 bytes) 0x0001
    cmd_val[0] = 0x01;
    cmd_val[1] = 0x0;
  }
  send_command(cmd, cmd_val, cmd_val_len);
}
  
void ld2412_parse_command_ack_frame(const uint8_t* frame_data) {
  // int intra_frame_data_length = frame_data[4] + (frame_data[5] << 8);

  // Read Firmware Version 
  // Command word: 2 bytes 0x01A0
  // Return value: 2-bytes ACK status (0 successful, 1 failed) + 2-bytes firmware type (0x2412)+2-bytes major
  //                version number+4-bytes minor version number
  if (frame_data[6] == 0xA0 && frame_data[7] == 0x01 && frame_data[8] == 0x0 && frame_data[9] == 0x0) {
    ESP_LOGI(TAG, "LD2412 firmware type: %02X.%02X", frame_data[11], frame_data[10]);
    ESP_LOGI(TAG, "LD2412 firmware version: V%02X.%02X.%02X.%02X.%02X.%02X", frame_data[13], frame_data[12], frame_data[17], frame_data[16], frame_data[15], frame_data[14]);
  } 
}

void read_firmware_version(void) {

  // 2.2.15 Read firmware version command
  // Command word (2 bytes) 0x00A0
  // Command value None
  uint8_t cmd[2] = {0xA0, 0x00};
  uint8_t cmd_val[2] = {};

  control_config_mode(true);
  vTaskDelay(150/portTICK_PERIOD_MS);

  // ESP_LOGI(TAG, "Time sent firmware read command: %lld us", esp_timer_get_time());
  send_command(cmd, cmd_val, 0);
  vTaskDelay(180/portTICK_PERIOD_MS);
  
  control_config_mode(false);
  vTaskDelay(150/portTICK_PERIOD_MS);
}

void control_engineering_mode(void) {

  current_engineering_mode = !current_engineering_mode;

  // Command word (2 bytes) 0x0062 or 0x0063
  // Command value None
  uint8_t cmd[2] = {current_engineering_mode ? 0x62 : 0x63, 0x00};
  uint8_t cmd_val[2] = {};

  control_config_mode(true);
  vTaskDelay(150/portTICK_PERIOD_MS);

  // ESP_LOGI(TAG, "Time sent engineering mode control: %lld us", esp_timer_get_time());
  send_command(cmd, cmd_val, 0);
  vTaskDelay(200/portTICK_PERIOD_MS);
  
  control_config_mode(false);
  vTaskDelay(150/portTICK_PERIOD_MS);
}