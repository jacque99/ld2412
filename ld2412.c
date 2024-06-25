#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include <esp_timer.h>

#include "ld2412.h"

static const char *TAG = "LD2412DEBUG";

struct ld2412_t {
    ld2412_config_t config;
    /* no other state to store for now */
};

uint8_t out_buf[UART_SERIAL_BUF_SZIE];

esp_err_t ld2412_init(const ld2412_config_t *config, ld2412_handle_t *out_handle) {
    struct ld2412_t *result = calloc(1, sizeof(struct ld2412_t));
    if (result == NULL) {
        return ESP_ERR_NO_MEM;
    }
    result->config = *config;
    *out_handle = result;

    return ESP_OK;
}

esp_err_t ld2412_deinit(ld2412_handle_t handle)
{
    free(handle);
    return ESP_OK;
}

void receive_task(void *arg) {

  uint8_t* rxbuf = (uint8_t*) malloc(UART_SERIAL_BUF_SZIE);
  // Check for available data
  // ESP_ERROR_CHECK(uart_get_buffered_data_len(handle->config.uart_port, (size_t*)&bytes_available));

  while (1) {
    // Read data from serial buffer
    const int rx_bytes = uart_read_bytes(RADAR_UART_PORT_NUMBER, rxbuf, 
                              21, 1000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Frame length: '%d'", rx_bytes);

    if (rx_bytes > 0) {
      rxbuf[rx_bytes] = 0;
      int intra_frame_data_length = rxbuf[4] + (rxbuf[5] << 8);
      ESP_LOGI(TAG, "Intra frame data length: '%d'", intra_frame_data_length);

      if ((rxbuf[0] == 0xF4) && (rxbuf[1] == 0xF3) && (rxbuf[2] == 0xF2) && (rxbuf[3] == 0xF1) &&                                      // Target Data Header  
        (rxbuf[rx_bytes-4] == 0xF8) && (rxbuf[rx_bytes-3] == 0xF7) && (rxbuf[rx_bytes-2] == 0xF6) && (rxbuf[rx_bytes-1] == 0xF5))      // Target Data End
      {
        ESP_LOGI(TAG, "TARGET DATA Frame header: '%d'", rxbuf[0]);
        for (int i=0; i<intra_frame_data_length; i++) {
          out_buf[i] = rxbuf[i+6];                                                                                // Read after Frame Header 4 bytes + Intra frame data length 2 bytes = 6
          // ESP_LOGI(TAG, "Intra frame data: '%d.%02X'", i, out_buf[i]);
        }

        ESP_LOGI(TAG, "Data type: %02X", out_buf[0]);
        ESP_LOGI(TAG, "Target state: %02X", out_buf[2]);
        ESP_LOGI(TAG, "Moving target [cm]: %d", out_buf[3] + (out_buf[4] << 8));
        ESP_LOGI(TAG, "Moving target energy: %d", out_buf[5]);
        ESP_LOGI(TAG, "Stationary target [cm]: %d", out_buf[6] + (out_buf[7] << 8));
        ESP_LOGI(TAG, "Stationary target energy: %d", out_buf[8]);
        ESP_LOGI(TAG, "End byte: %02X", out_buf[9]);

      }
      else if ((rxbuf[0] == 0xFD) && (rxbuf[1] == 0xFC) && (rxbuf[2] == 0xFB) && (rxbuf[3] == 0xFA) &&                                 // Radar ACK Header
        (rxbuf[rx_bytes-4] == 0x04) && (rxbuf[rx_bytes-3] == 0x03) && (rxbuf[rx_bytes-2] == 0x02) && (rxbuf[rx_bytes-1] == 0x01))      // Radar ACK End
      {
        ESP_LOGI(TAG, "ACK DATA Frame header: '%d'", rxbuf[0]);
        for (int i=0; i<intra_frame_data_length; i++) {
          out_buf[i] = rxbuf[i+6];                                                                                // Read after Frame Header 4 bytes + Intra frame data length 2 bytes = 6
          ESP_LOGI(TAG, "Intra frame data: '%d.%02X'", i, out_buf[i]);
        }
      }
    }
  }
  free(rxbuf);
}
void send_frame_header(uart_port_t uart_port) {
  uint8_t frame_header[4] = {0xFD, 0xFC, 0xFB, 0xFA};
  uart_write_bytes(uart_port, frame_header, sizeof(frame_header));
}

void send_frame_end(uart_port_t uart_port) {
  uint8_t frame_end[4] = {0x04, 0x03, 0x02, 0x01};
  uart_write_bytes(uart_port, frame_end, sizeof(frame_end));
}

int send_command(ld2412_handle_t handle, uint8_t *command_str, uint8_t *command_val, int command_val_len) {
  int intra_frame_data_length = 2;

  send_frame_header(handle->config.uart_port);

  // Clear receive buffer
  uart_flush(handle->config.uart_port);

  uint8_t intra_frame_data[UART_SERIAL_BUF_SZIE] = {};
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
  int tx_bytes = uart_write_bytes(handle->config.uart_port, intra_frame_data, intra_frame_data_length+2);
  ESP_LOGI(TAG, "TX bytes: '%d'", tx_bytes);

  send_frame_end(handle->config.uart_port);

  return tx_bytes; 
}

int control_config_mode(ld2412_handle_t handle, bool enable) {
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

  return (send_command(handle, cmd, cmd_val, cmd_val_len));
}

bool read_firmware_version(ld2412_handle_t handle, uint8_t *out_buf) {

  if (control_config_mode(handle, true) == 6) {
    uint8_t cmd[2] = {0xA0, 0x00};
    uint8_t cmd_val[2] = {};
  
    ESP_LOGI(TAG, "ENTERED COMMAND MODE");

    if (send_command(handle, cmd, cmd_val, 0) == 4) {
      ESP_LOGI(TAG, "SENT FIRMWARE READ COMMAND");
      if (control_config_mode(handle, false) == 4) {
        ESP_LOGI(TAG, "LEAVED COMMAND MODE");
        return true;
      }
    }
  }
  control_config_mode(handle, false);
  return false;
}