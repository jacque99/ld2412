/**
 ******************************************************************************
 * @file    		:  uart_config.h
 * @author  		:  Lhagva
 * @version 		:  v.1.0
 * @date    		:  Jul 3, 2024
 * @brief   		:  UART events and UART RX, TX tasks
 *
 * UART events UART_DATA reads serial buffer and send data to queue uartRxStore_queue.
 * uart_reception_task starts when uartRxStore_queue is filled. It parses recieved frame data and 
 * send parsed data to system_queue for further processing.
 * uart_transmission_task starts when uartTx_queue is filled, and send data to serial.   
 * 
 ******************************************************************************/

#ifndef UART_CONFIG_H_
#define UART_CONFIG_H_


/* INCLUDES ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

/* MACROS --------------------------------------------------------------------*/
#define UART_PORT_NUMBER		        UART_NUM_1
#define UART_TXD_PIN                    5
#define UART_RXD_PIN                    4
#define TX_BUF_SIZE                     1024
#define RX_BUF_SIZE                     1024

#define SYSTEM_BUFFER_SIZE				3
/* PRIVATE STRUCTRES ---------------------------------------------------------*/
typedef struct
{
	uint8_t packet_size;
	uint16_t data[SYSTEM_BUFFER_SIZE];
}system_packet;

/* STRUCTURES & TYPEDEFS -----------------------------------------------------*/
typedef struct
{
	uint8_t uart_rxBuffer[RX_BUF_SIZE];
	uint8_t uart_txBuffer[TX_BUF_SIZE];
	uint8_t uart_rxPacketSize;
	uint8_t uart_txPacketSize;
	union
	{
		uint8_t all;
		struct
		{
			uint8_t reserved		:7,
					rxPacket		:1;
		}flags;
	}uart_status;
}uartHandler_t;

/* VARIABLES -----------------------------------------------------------------*/
extern QueueHandle_t 			system_queue;
extern QueueHandle_t 			uartTx_queue;

extern uartHandler_t	 		hUart;

/* FUNCTIONS DECLARATION -----------------------------------------------------*/
/**
 * @brief initialize UART handler
 *
 */
void 	uart_buffer_init		(void);

/**
 * @brief configure UART
 * Assign UART PORT and set uartRx_queue as UART receive event queue handle
 * Configure UART parameters defined in uart_config
 * Define pin numbers for UART PORT  
 */
void 	uart_config				(void);

/**
 * @brief UART event task. Here UART RX callback takes place. This task should be started in the main
 *
 * @param pvParameters
 */
void 	uart_event_task			(void *pvParameters);

/**
 * @brief UART TX function. The transmission starts once the related queue is filled.
 *
 * @param pvParameters
 */
void 	uart_transmission_task	(void *pvParameters);

/**
 * @brief UART RX function. The reception starts once the related queue is filled.
 *
 * @param pvParameters
 */
void 	uart_reception_task	    (void *pvParameters);

#endif /* UART_CONFIG_H_ */