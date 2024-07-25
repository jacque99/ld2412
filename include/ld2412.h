#pragma once

#include "esp_err.h"
#include <driver/uart.h>
#include <driver/gpio.h>

/**
 * @file ld2412.h
 * @brief Driver for the LD2410/2412 Presense detector
 *
 * This driver is used to:
 * - Receives periodic data from Radar UART using the UART event handler.
 * - Send ACK commands with send_command()
 * - Receives ACK commands and parse data
 */

#ifdef __cplusplus
extern "C" {
#endif

/* MACROS --------------------------------------------------------------------*/
#define RADAR_MAX_FRAME_LENGTH 	        51

/* FUNCTIONS DECLARATION -----------------------------------------------------*/
/**
 * @brief Send command to Radar
 * This function constructs data frame including the header and end, and sends to the TX queue.
 * @param command_str Command string
 * @param command_value Command value
 * @param command_val_len Command value's length
 *
 */
void send_command(uint8_t *command_str, uint8_t *command_val, int command_val_len);

/**
 * @brief Enable/Close configuation mode
 *
 * @param enable 
 *
 */
void control_config_mode(bool enable);

/**
 * @brief Enable/Close engineering mode command
 *        Current implementation toggles engineering mode 
 * @param void
 *
 */
void control_engineering_mode(void);

/**
 * @brief Send command for read firmware version from Radar
 *
 * @param void
 *
 */
void read_firmware_version(void);

/**
 * @brief Set the serial port baud rate
 *
 * @param command_val The command value to set baud rate
 *
 */
void set_baud_rate(uint8_t *command_val);

void restart_module(void);

/**
 * @brief Parse Target data frame received from Radar and return the target distance
 *
 * @param frame_data The input frame data from which the Target data to be extracted.
 * @param movement_distance The array where the moving target distance will be stored.
 * @param static_distance The array where the stationary target distance will be stored.
 * 
 * @return The target state value as a number.
*/
uint8_t ld2412_parse_target_data_frame(const uint8_t* frame_data, int16_t* movement_distance, int16_t* static_distance);

/**
 * @brief Parse command ACK frame received from Radar
 *
 * @param frame_data The input frame data from which the command ACK to be extracted.
 *
 */
void ld2412_parse_command_ack_frame(const uint8_t* frame_data);

#ifdef __cplusplus
}
#endif /* End of CPP guard */