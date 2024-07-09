#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include <esp_timer.h>

#include "uart_config.h"
#include "ld2412.h"

static const char *TAG = "LD2412DEBUG";

void send_frame_header() {
  hUart.uart_txBuffer[0] = 0xFD;
  hUart.uart_txBuffer[1] = 0xFC;
  hUart.uart_txBuffer[2] = 0xFB;
  hUart.uart_txBuffer[3] = 0xFA;
  hUart.uart_txPacketSize = 4; 
  xQueueSendToBack(uartTx_queue, &hUart, portMAX_DELAY);
  // uint8_t frame_header[4] = {0xFD, 0xFC, 0xFB, 0xFA};
  // uart_write_bytes(uart_port, frame_header, sizeof(frame_header));
}

void send_frame_end() {
  hUart.uart_txBuffer[0] = 0x04;
  hUart.uart_txBuffer[1] = 0x03;
  hUart.uart_txBuffer[2] = 0x02;
  hUart.uart_txBuffer[3] = 0x01;
  hUart.uart_txPacketSize = 4; 
  xQueueSendToBack(uartTx_queue, &hUart, portMAX_DELAY);
  // uint8_t frame_end[4] = {0x04, 0x03, 0x02, 0x01};
  // uart_write_bytes(uart_port, frame_end, sizeof(frame_end));
}

void send_command(uint8_t *command_str, uint8_t *command_val, int command_val_len) {
  int intra_frame_data_length = 2;

  send_frame_header();

  uint8_t intra_frame_data[RADAR_MAX_FRAME_LENGTH] = {};
  ESP_LOGI(TAG, "command_val: '%d.%d'", command_val[0], command_val[1]);

  if (command_val[0] != '\0') 
    intra_frame_data_length += command_val_len;

  // intra frame data length
  intra_frame_data[0] = (uint8_t)(intra_frame_data_length & 0xFF);
  intra_frame_data[1] = (uint8_t)((intra_frame_data_length >> 8) & 0xFF);
  // // command word bytes
  intra_frame_data[2] = command_str[0];
  intra_frame_data[3] = command_str[1];
  // command value bytes
  if (command_val[0] != '\0') {
    for (int i = 4; i < command_val_len+4; i++)
    {
      intra_frame_data[i] = command_val[i];
    }
  }
  ESP_LOGI(TAG, "intra_frame_data_length: '%d'", intra_frame_data_length);
  
  hUart.uart_txPacketSize = sizeof(intra_frame_data);
  memcpy(hUart.uart_txBuffer, intra_frame_data, sizeof(intra_frame_data)); 
  xQueueSendToBack(uartTx_queue, &hUart, portMAX_DELAY);
  // int tx_bytes = uart_write_bytes(handle->config.uart_port, intra_frame_data, intra_frame_data_length+2);
  // ESP_LOGI(TAG, "TX bytes: '%d'", tx_bytes);

  send_frame_end();
}

void control_config_mode(bool enable) {
  uint8_t cmd[2] = {0xFF, 0x0};
  uint8_t cmd_val[2] = {};
  int cmd_val_len = 2;
  if (enable) {
    // Enable Configuration Command 2.2.1
    // Intra-frame data length 2 bytes, Value is 0x04
    // Command word (2 bytes) 0x00FF
    // Command value (2 bytes) 0x0001
    cmd_val[0] = 0x01;
    cmd_val[1] = 0x0;
  } else {
    // End Command Configuration 2.2.2 
    // Intra-frame data length = 2 bytes, Value is 0x02
    // Command word (2 bytes) 0x00FE
    // Command value None
    cmd[0] = 0xFE;
    cmd_val_len = 0;
  }
  send_command(cmd, cmd_val, cmd_val_len);
}

// int16_t ld2412_parse_target_frame(const uint8_t* frame, uint8_t* target_state) {
//   ESP_LOGI(TAG, "target frame: '%02X'", frame[0]);
//   if (frame[6] == 0x02 && frame[7] == 0xAA && frame[17] == 0x55 && frame[18] == 0x00) {
//     // Normal mode target data 
//     *target_state = 8;//(uint8_t)frame[8];
//     return (frame[3] + (frame[4] << 8));
//   } else if (frame[6] == 0x01 && frame[7] == 0xAA && frame[45] == 0x55 && frame[46] == 0x00) {
//     // Engineering mode target data 
//     *target_state = 9; //(uint8_t)frame[8];
//     return (frame[3] + (frame[4] << 8));
//   }
//   return -1; // Return -1.0 if target distance value not found
// }

// bool read_firmware_version(ld2412_handle_t handle, uint8_t *out_buf) {

//   if (control_config_mode(handle, true) == 6) {
//     uint8_t cmd[2] = {0xA0, 0x00};
//     uint8_t cmd_val[2] = {};
  
//     ESP_LOGI(TAG, "ENTERED COMMAND MODE");

//     if (send_command(handle, cmd, cmd_val, 0) == 4) {
//       ESP_LOGI(TAG, "SENT FIRMWARE READ COMMAND");
//       if (control_config_mode(handle, false) == 4) {
//         ESP_LOGI(TAG, "LEAVED COMMAND MODE");
//         return true;
//       }
//     }
//   }
//   control_config_mode(handle, false);
//   return false;
// }