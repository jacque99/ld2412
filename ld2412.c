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

uint8_t ld2412_parse_target_data_frame(const uint8_t* frame_data, int16_t* movement_distance, int16_t* static_distance) {
  if (frame_data[6] == 0x02 && frame_data[7] == 0xAA && frame_data[15] == 0x55) { // && frame_data[16] == 0x00) { // DL 24.07.08 sometimes value of 0xF1, 0x9, and 0x1 comes into 17th Check byte, instead of 0x00
    // Normal mode target data 
    *movement_distance = frame_data[9] + (frame_data[10] << 8);
    // *movement_energy = frame_data[11];
    *static_distance = frame_data[12] + (frame_data[13] << 8);
    // *static_energy = frame_data[14];
    return frame_data[8]; // Normal mode
  } else if (frame_data[6] == 0x01 && frame_data[7] == 0xAA && frame_data[45] == 0x55 && frame_data[46] == 0x00) {
    // Engineering mode target data
    ESP_LOGI(TAG, "Engineering mode data length: %d", frame_data[4] + (frame_data[5] << 8));
    // TO DO Parse Engineering mode target data
    return frame_data[8]; // Engineering mode
  } else {
    return 100; // Return 100 if correct target frame not found
  }
}
void ld2412_parse_command_ack_frame(const uint8_t* frame_data) {
  // Enable engineering mode 
  // Command word: 2 bytes 0x0162
  // Return value: 2-bytes ACK status (0 successful, 1 failed)
  if (frame_data[6] == 0x62 && frame_data[7] == 0x01 && frame_data[8] == 0x0 && frame_data[9] == 0x0) {
    ESP_LOGI(TAG, "Successfully Enabled engineering mode");
  } 
  // Close engineering mode 
  // Command word: 2 bytes 0x0163
  // Return value: 2-bytes ACK status (0 successful, 1 failed)
  if (frame_data[6] == 0x63 && frame_data[7] == 0x01 && frame_data[8] == 0x0 && frame_data[9] == 0x0) {
    ESP_LOGI(TAG, "Successfully Closed engineering mode");
  }

  // Read Firmware Version 
  // Command word: 2 bytes 0x01A0
  // Return value: 2-bytes ACK status (0 successful, 1 failed) + 2-bytes firmware type (0x2412)+2-bytes major
  //                version number+4-bytes minor version number
  if (frame_data[6] == 0xA0 && frame_data[7] == 0x01 && frame_data[8] == 0x0 && frame_data[9] == 0x0) {
    ESP_LOGI(TAG, "LD2412 firmware type: %02X.%02X", frame_data[11], frame_data[10]);
    ESP_LOGI(TAG, "LD2412 firmware version: V%02X.%02X.%02X.%02X.%02X.%02X", frame_data[13], frame_data[12], frame_data[17], frame_data[16], frame_data[15], frame_data[14]);
  } 
  // Set serial port baud rate 
  // Command word: 2 bytes 0x01A1
  // Return value: 2-bytes ACK status (0 successful, 1 failed)
  if (frame_data[6] == 0xA1 && frame_data[7] == 0x01 && frame_data[8] == 0x0 && frame_data[9] == 0x0) {
    ESP_LOGI(TAG, "Successfully Set serial baud rate, please restart the module");
  }
}

/**
 * @brief Enable/Close engineering mode command
 *        Current implementation toggles engineering mode 
 * @param void
 *
 */
void control_engineering_mode(void) {

  current_engineering_mode = !current_engineering_mode;

  // Command word (2 bytes) 0x0062 or 0x0063
  // Command value None
  uint8_t cmd[2] = {current_engineering_mode ? 0x62 : 0x63, 0x00};
  uint8_t cmd_val[2] = {};

  control_config_mode(true);
  vTaskDelay(150/portTICK_PERIOD_MS);

  send_command(cmd, cmd_val, 0);
  vTaskDelay(200/portTICK_PERIOD_MS);
  
  control_config_mode(false);
  vTaskDelay(150/portTICK_PERIOD_MS);
}

/**
 * @brief Send command for read firmware version from Radar
 *
 * @param void
 *
 */
void read_firmware_version(void) {
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

/**
 * @brief Set the serial port baud rate
 *
 * @param void
 *
 */
void set_baud_rate(uint8_t *command_val) {
  // Command word (2 bytes) 0x00A1
  uint8_t cmd[2] = {0xA1, 0x00};

  control_config_mode(true);
  vTaskDelay(150/portTICK_PERIOD_MS);

  ESP_LOGI(TAG, "Time sent set baud rate command: %lld us", esp_timer_get_time());
  send_command(cmd, command_val, 2);
  vTaskDelay(180/portTICK_PERIOD_MS);
  
  control_config_mode(false);
  vTaskDelay(150/portTICK_PERIOD_MS);

  restart_module();
}

void restart_module(void) {
  // Command word (2 bytes) 0x00A3
  // Command value None
  uint8_t cmd[2] = {0xA3, 0x00};
  uint8_t cmd_val[2] = {};

  control_config_mode(true);
  vTaskDelay(150/portTICK_PERIOD_MS);

  send_command(cmd, cmd_val, 0);
  vTaskDelay(180/portTICK_PERIOD_MS);
  
  control_config_mode(false);
  vTaskDelay(150/portTICK_PERIOD_MS);
}