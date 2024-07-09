
/**
 ******************************************************************************
 * @file    		:  uart_config.h
 * @author  		:  
 * @version 		:  v.1.0
 * @date    		:  Jul 3, 2024
 * @brief   		:
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

#define SYSTEM_BUFFER_SIZE				5
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
extern QueueHandle_t 			uartRxStore_queue;
extern QueueHandle_t 			uartTx_queue;

extern uartHandler_t	 		hUart;

/* FUNCTIONS DECLARATION -----------------------------------------------------*/
void 	uart_buffer_init		(void);
void 	uart_config				(void);
void 	uart_event_task			(void *pvParameters);
void 	uart_transmission_task	(void *pvParameters);
void 	uart_reception_task	    (void *pvParameters);

#endif /* UART_CONFIG_H_ */
