/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include "uart_config.h"
#include "ld2412.h"

static const char *TAG = "UART";

/* VARIABLES -----------------------------------------------------------------*/
QueueHandle_t uartRx_queue;
QueueHandle_t uartRxStore_queue;
QueueHandle_t uartTx_queue;

uartHandler_t hUart;
/* FUNCTION PROTOTYPES -------------------------------------------------------*/
/**
 * @brief initialize UART handler
 *
 */
void uart_buffer_init(void) {
	memset(&hUart, 0,sizeof(uartHandler_t));
}

void uart_config(void) {
  const uart_config_t uart_config =
  {
    // Baud rate must be set to default 115200
    .baud_rate  = 115200,
    .data_bits  = UART_DATA_8_BITS,
    .parity     = UART_PARITY_DISABLE,
    .stop_bits  = UART_STOP_BITS_1,
    .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };
  // Install UART driver, and get the queue.
  // queue_size: 20                       UART event queue size/depth.
  // uart_queue: uartRx_queue             UART event queue handle (out param).
  // intr_alloc_flags: 0		    Flags used to allocate the interrupt. 
  ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUMBER, RX_BUF_SIZE * 2, TX_BUF_SIZE * 2, 20, &uartRx_queue, 0));
  // Configure UART parameters
  ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUMBER, &uart_config));
  // Set UART pins
  ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUMBER, UART_TXD_PIN, UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  uart_buffer_init();

  uartTx_queue = xQueueCreate(10, sizeof(uartHandler_t));
  uartRxStore_queue	= xQueueCreate(10, sizeof(uartHandler_t));
}

/**
 * @brief UART TX function. The transmission starts once the related queue is filled.
 *
 * @param pvParameters
 */
void uart_transmission_task(void *pvParameters) {
	while(1) {
		if(xQueueReceive(uartTx_queue, (void *)&hUart, portMAX_DELAY)) {

			uart_write_bytes(UART_PORT_NUMBER, hUart.uart_txBuffer, hUart.uart_txPacketSize);

			vTaskDelay(150/portTICK_PERIOD_MS);
		}
	}
}

/**
 * @brief UART RX function. The reception starts once the related queue is filled.
 *
 * @param pvParameters
 */
void uart_reception_task(void *pvParameters) {
  uartHandler_t uartHandler = {0};

  system_packet system_buffer = {0};

  system_queue = xQueueCreate(10, sizeof(system_packet));
  for(;;) {
    // Waiting for UART packet to get received.
    if(xQueueReceive(uartRxStore_queue, (void * )&uartHandler, portMAX_DELAY)) {
      ESP_LOGI(TAG, "Packet size: '%d'", hUart.uart_rxPacketSize);
      // int intra_frame_data_length = hUart.uart_rxBuffer[4] + (hUart.uart_rxBuffer[5] << 8);
      for (int i=0; i<hUart.uart_rxPacketSize; i++) {
        ESP_LOGI(TAG, "Frame data: '%d.%02X'", i, hUart.uart_rxBuffer[i]);
      }
      // Target Data Header
      if ((hUart.uart_rxBuffer[0] == 0xF4) && (hUart.uart_rxBuffer[1] == 0xF3) && (hUart.uart_rxBuffer[2] == 0xF2) && (hUart.uart_rxBuffer[3] == 0xF1) &&
        // Target Data End
        (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-4] == 0xF8) && (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-3] == 0xF7) && (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-2] == 0xF6) && (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-1] == 0xF5))
      {
        ESP_LOGI(TAG, "TARGET DATA Frame header: '%02X'", hUart.uart_rxBuffer[0]);
        if (hUart.uart_rxBuffer[6] == 0x02 && hUart.uart_rxBuffer[7] == 0xAA && hUart.uart_rxBuffer[15] == 0x55) { //&& hUart.uart_rxBuffer[16] == 0x00) { // DL 24.07.08 sometimes 0xF3 comes instead of check byte 0x00 
          // Normal mode target data 
          // Read after Frame Header 4 bytes + Intra frame data length 2 bytes = 6
          system_buffer.data[0] = hUart.uart_rxBuffer[8];                                     // Target state 
          system_buffer.data[1] = hUart.uart_rxBuffer[9] + (hUart.uart_rxBuffer[10] << 8);    // Moving target 
          system_buffer.data[2] = hUart.uart_rxBuffer[11];                                    // Moving target energy 
          system_buffer.data[3] = hUart.uart_rxBuffer[12] + (hUart.uart_rxBuffer[13] << 8);   // Stationary target 
          system_buffer.data[4] = hUart.uart_rxBuffer[14];                                    // Stationary target energy
          
          system_buffer.packet_size = 7;
          xQueueSendToBack(system_queue, &system_buffer, portMAX_DELAY);

        } else if (hUart.uart_rxBuffer[6] == 0x01 && hUart.uart_rxBuffer[7] == 0xAA && hUart.uart_rxBuffer[45] == 0x55 && hUart.uart_rxBuffer[46] == 0x00) {
          // Engineering mode target data 
          // Read after Frame Header 4 bytes + Intra frame data length 2 bytes = 6
          system_buffer.data[0] = hUart.uart_rxBuffer[8];                                     // Target state 
          system_buffer.data[1] = hUart.uart_rxBuffer[9] + (hUart.uart_rxBuffer[10] << 8);    // Moving target 
          system_buffer.data[2] = hUart.uart_rxBuffer[11];                                    // Moving target energy 
          system_buffer.data[3] = hUart.uart_rxBuffer[12] + (hUart.uart_rxBuffer[13] << 8);   // Stationary target 
          system_buffer.data[4] = hUart.uart_rxBuffer[14];                                    // Stationary target energy
          
          system_buffer.packet_size = 47;
          xQueueSendToBack(system_queue, &system_buffer, portMAX_DELAY);
        }
      } else 
      // Comamnd Data Header
      if ((hUart.uart_rxBuffer[0] == 0xFD) && (hUart.uart_rxBuffer[1] == 0xFC) && (hUart.uart_rxBuffer[2] == 0xFB) && (hUart.uart_rxBuffer[3] == 0xFA) &&
        // Comamnd Data End
        (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-4] == 0x04) && (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-3] == 0x03) && (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-2] == 0x02) && (hUart.uart_rxBuffer[hUart.uart_rxPacketSize-1] == 0x01))
      {
        // Parse command frame data with ACK
        // ld2412_parse_command_frame(hUart.uart_rxBuffer);
      }
    }
  }
}

// static void receive_task(void *arg) {

//   uint8_t* rxbuf = (uint8_t*) malloc(UART_BUF_SZIE);
//   // Check for available data
//   // ESP_ERROR_CHECK(uart_get_buffered_data_len(handle->config.uart_port, (size_t*)&bytes_available));

//   while (1) {
//     // Read data from serial buffer
//     const int rx_bytes = uart_read_bytes(UART_PORT_NUMBER, rxbuf, 
//                               128, 1000 / portTICK_PERIOD_MS);

//     ESP_LOGI(TAG, "Frame length: '%d'", rx_bytes);

//     if (rx_bytes > 0) {
//       rxbuf[rx_bytes] = 0;
//       int intra_frame_data_length = rxbuf[4] + (rxbuf[5] << 8);
//       ESP_LOGI(TAG, "Intra frame data length: '%d'", intra_frame_data_length);

//       if ((rxbuf[0] == 0xF4) && (rxbuf[1] == 0xF3) && (rxbuf[2] == 0xF2) && (rxbuf[3] == 0xF1) &&                                      // Target Data Header  
//         (rxbuf[rx_bytes-4] == 0xF8) && (rxbuf[rx_bytes-3] == 0xF7) && (rxbuf[rx_bytes-2] == 0xF6) && (rxbuf[rx_bytes-1] == 0xF5))      // Target Data End
//       {
//         ESP_LOGI(TAG, "TARGET DATA Frame header: '%d'", rxbuf[0]);
//         for (int i=0; i<intra_frame_data_length; i++) {
//           out_buf[i] = rxbuf[i+6];                                                                                // Read after Frame Header 4 bytes + Intra frame data length 2 bytes = 6
//           // ESP_LOGI(TAG, "Intra frame data: '%d.%02X'", i, out_buf[i]);
//         }

//         ESP_LOGI(TAG, "Data type: %02X", out_buf[0]);
//         ESP_LOGI(TAG, "Target state: %02X", out_buf[2]);
//         ESP_LOGI(TAG, "Moving target [cm]: %d", out_buf[3] + (out_buf[4] << 8));
//         ESP_LOGI(TAG, "Moving target energy: %d", out_buf[5]);
//         ESP_LOGI(TAG, "Stationary target [cm]: %d", out_buf[6] + (out_buf[7] << 8));
//         ESP_LOGI(TAG, "Stationary target energy: %d", out_buf[8]);
//         ESP_LOGI(TAG, "End byte: %02X", out_buf[9]);

//       }
//       else if ((rxbuf[0] == 0xFD) && (rxbuf[1] == 0xFC) && (rxbuf[2] == 0xFB) && (rxbuf[3] == 0xFA) &&                                 // Radar ACK Header
//         (rxbuf[rx_bytes-4] == 0x04) && (rxbuf[rx_bytes-3] == 0x03) && (rxbuf[rx_bytes-2] == 0x02) && (rxbuf[rx_bytes-1] == 0x01))      // Radar ACK End
//       {
//         ESP_LOGI(TAG, "ACK DATA Frame header: '%d'", rxbuf[0]);
//         for (int i=0; i<intra_frame_data_length; i++) {
//           out_buf[i] = rxbuf[i+6];                                                                                // Read after Frame Header 4 bytes + Intra frame data length 2 bytes = 6
//           ESP_LOGI(TAG, "Intra frame data: '%d.%02X'", i, out_buf[i]);
//         }
//       }
//     }
//   }
//   free(rxbuf);
// }

/**
 * @brief UART event task. Here UART RX callback takes place. This task should be started in the main
 *
 * @param pvParameters
 */
void uart_event_task(void *pvParameters) {
  uart_event_t event;
  // size_t buffered_size;
  uint8_t* dtmp = (uint8_t*) malloc(RX_BUF_SIZE);
  for(;;) {
    //Waiting for UART event.
    if(xQueueReceive(uartRx_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
      bzero(dtmp, RX_BUF_SIZE);
      ESP_LOGI(TAG, "uart[%d] event:", UART_PORT_NUMBER);
      switch(event.type) {
      //Event of UART receving data
      /*We'd better handler data event fast, there would be much more data events than
      other types of events. If we take too much time on data event, the queue might
      be full.*/
      case UART_DATA:
        uart_read_bytes(UART_PORT_NUMBER, hUart.uart_rxBuffer, event.size, portMAX_DELAY);

        hUart.uart_rxPacketSize = event.size;
        hUart.uart_status.flags.rxPacket = 1;

        xQueueSendToBack(uartRxStore_queue, &hUart, portMAX_DELAY);
        break;
      //Event of HW FIFO overflow detected
      case UART_FIFO_OVF:
        ESP_LOGI(TAG, "hw fifo overflow");
        // If fifo overflow happened, you should consider adding flow control for your application.
        // The ISR has already reset the rx FIFO,
        // As an example, we directly flush the rx buffer here in order to read more data.
        uart_flush_input(UART_PORT_NUMBER);
        xQueueReset(uartRx_queue);
        break;
        //Event of UART ring buffer full
      case UART_BUFFER_FULL:
        ESP_LOGI(TAG, "ring buffer full");
        // If buffer full happened, you should consider encreasing your buffer size
        // As an example, we directly flush the rx buffer here in order to read more data.
        uart_flush_input(UART_PORT_NUMBER);
        xQueueReset(uartRx_queue);
        break;
      //Event of UART RX break detected
      case UART_BREAK:
        ESP_LOGI(TAG, "uart rx break");
        break;
      //Event of UART parity check error
      case UART_PARITY_ERR:
        ESP_LOGI(TAG, "uart parity error");
        break;
      //Event of UART frame error
      case UART_FRAME_ERR:
        ESP_LOGI(TAG, "uart frame error");
        break;

      // //UART_PATTERN_DET
      // case UART_PATTERN_DET:
      //   uart_get_buffered_data_len(UART_PORT_NUMBER, &buffered_size);
      //   int pos = uart_pattern_pop_pos(UART_PORT_NUMBER);
      //   ESP_LOGI(UART_DEBUG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
      //   if (pos == -1) {
      //       // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
      //       // record the position. We should set a larger queue size.
      //       // As an example, we directly flush the rx buffer here.
      //       uart_flush_input(UART_PORT_NUMBER);
      //   } else {
      //       uart_read_bytes(UART_PORT_NUMBER, dtmp, pos, 100 / portTICK_PERIOD_MS);
      //       uint8_t pat[PATTERN_AT_COUNT + 1];
      //       memset(pat, 0, sizeof(pat));
      //       uart_read_bytes(UART_PORT_NUMBER, pat, PATTERN_AT_COUNT, 100 / portTICK_PERIOD_MS);
      //       ESP_LOGI(UART_DEBUG, "read data: %s", dtmp);
      //       ESP_LOGI(UART_DEBUG, "read pat : %s", pat);
      //   }
      //   break;
      //Others
      default:
        ESP_LOGI(TAG, "uart event type: %d", event.type);
        break;
      }
    }
  }
  free(dtmp);
  dtmp = NULL;
  vTaskDelete(NULL);
}
