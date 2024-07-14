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
#define RADAR_MAX_FRAME_LENGTH 	        41

/* FUNCTIONS DECLARATION -----------------------------------------------------*/
void send_command(uint8_t *command_str, uint8_t *command_val, int command_val_len);
void control_config_mode(bool enable);
void control_engineering_mode(void);

/**
 * @brief Send command for read firmware version from Radar
 *
 * @param void
 *
 */
void read_firmware_version(void);

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