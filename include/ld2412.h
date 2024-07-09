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
#define RADAR_MAX_FRAME_LENGTH 	        40

/* FUNCTIONS DECLARATION -----------------------------------------------------*/

void send_frame_header(void);
void send_frame_end(void);
void control_config_mode(bool enable);
void send_command(uint8_t *command_str, uint8_t *command_val, int command_val_len);

/**
 * @brief Extracts the target distance value from the provided frame data and also identifies the target state.
 *
 * This function extracts the distance value from the target frame and identifies the target state.
 * The target distance_value is a integer value representing the distance.
 *
 * @param frame The input frame data from which the distance is to be extracted.
 * @param target_state The integer where the target state will be stored.
 *
 * @return The extracted distance value as a number in cm, or -1 if the distance value is not found.
 *
 */
// int16_t ld2412_parse_target_frame(const uint8_t* frame, uint8_t* target_state);

#ifdef __cplusplus
}
#endif /* End of CPP guard */