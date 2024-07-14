
/**
 ******************************************************************************
 * @file    		:  button_config.h
 * @author  		:  
 * @version 		:  v.1.0
 * @date    		:  Jul 3, 2024
 * @brief   		:
 *
 ******************************************************************************/

#ifndef MAIN_BUTTON_CONFIG_H_
#define MAIN_BUTTON_CONFIG_H_

#include "iot_button.h"
#include "esp_idf_version.h"

#include "ld2412.h"

/* Most development boards have "boot" button attached to GPIO0.
 * You can also change this to another pin.
 */
#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32H2 || CONFIG_IDF_TARGET_ESP32C6
#define BOOT_BUTTON_NUM         9
#else
#define BOOT_BUTTON_NUM         0
#endif

#define BUTTON_ACTIVE_LEVEL     0

void button_init(uint32_t button_num);

#endif /* MAIN_BUTTON_CONFIG_H_ */
