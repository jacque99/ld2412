#pragma once

#include "esp_err.h"
#include <driver/uart.h>
#include <driver/gpio.h>

/**
 * @file ld2412.h
 * @brief Driver for the LD2410/2412 Presense detector
 *
 * To use this driver:
 * - Send ACK commands with send_command()
 */

#ifdef __cplusplus
extern "C" {
#endif

/* MACROS --------------------------------------------------------------------*/
#define RADAR_MAX_FRAME_LENGTH 	        51

/* FUNCTIONS DECLARATION -----------------------------------------------------*/
void send_command(uint8_t *command_str, uint8_t *command_val, int command_val_len);
void control_config_mode(bool enable);
/**
 * @brief Enable/Close engineering mode command
 *
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
 * @param void
 *
 */
void set_baud_rate(void);

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