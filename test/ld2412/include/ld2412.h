#pragma once

#include "esp_err.h"
#include <driver/uart.h>
#include <driver/gpio.h>

/**
 * @file ld2412.h
 * @brief Driver for the LD2410/2412 Presense detector
 *
 * To use this driver:
 * - Initialize the driver with ld2412_init()
 * - Call read_firmware_version() to get the firmware version from the RADAR
 * - Deinitialize the driver with ld2412_deinit()
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LD2412 driver configuration
 */
typedef struct {
    uart_port_t uart_port;  ///< UART port used to communicate
    int64_t last_ts;        ///< timestamp of the last sensor reading

} ld2412_config_t;

/**
 * @brief LD2412 driver handle
 */
typedef struct ld2412_t *ld2412_handle_t;

#define RADAR_UART_PORT_NUMBER          UART_NUM_1
#define RADAR_MAX_FRAME_LENGTH 	        40
//! 128 is the minimal value the UART driver will accept (at least on esp32)
#define UART_BUF_SIZE                   128
//! Response timeout between 15..120 ms at 115200 baud works reliable for all commands
// #define RADAR_UART_TIMEOUT_MS           100

/**
 * @brief Initialize the LD2412 driver
 *
 * @param config Pointer to the configuration struct. The driver makes a copy, so can point to a local variable.
 * @param[out] out_handle  Pointer to a variable to receive the driver handle.
 * @return esp_err_t  ESP_OK on success, ESP_ERR_NO_MEM if out of memory.
 */
esp_err_t ld2412_init(const ld2412_config_t *config, ld2412_handle_t *out_handle);

/**
 * @brief Deinitialize the LD2412 driver
 *
 * @param handle Driver handle obtained from ld2412_init(), or NULL
 * @return esp_err_t  ESP_OK on success.
 */
esp_err_t ld2412_deinit(ld2412_handle_t handle);

void send_frame_header(uart_port_t uart_port);
void send_frame_end(uart_port_t uart_port);
int send_command(ld2412_handle_t handle, uint8_t *command_str, uint8_t *command_val, int command_val_len);
void receive_task(void *arg);

int control_config_mode(ld2412_handle_t handle, bool enable);

/**
 * @brief Read the Firmware Version from the LD2412
 *
 * @param handle Driver handle obtained from ld2412_init()
 * @param out_buf Destination buffer for the FW Version message, has to be at least 12 bytes long
 * @return bool
 *      - true on success
 *      - false if the error
 */

bool read_firmware_version(ld2412_handle_t handle, uint8_t *out_buf);

#ifdef __cplusplus
}
#endif /* End of CPP guard */