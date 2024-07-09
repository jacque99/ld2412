#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include <esp_timer.h>

#include "uart_config.h"
#include "ld2412.h"

static const char *TAG = "LD2412DEMO";

/* VARIABLES -----------------------------------------------------------------*/
QueueHandle_t system_queue;
char targetString[10] = {0};

/* PRIVATE FUNCTIONS DECLARATION ---------------------------------------------*/
static void display_time_task(void*param);

/**
 * @brief 	Display timer task. Necessary to run once every 10ms
 *
 */
void display_time_task(void* param)
{
	// To keep track of the time the task last woke up
  // TickType_t xLastWakeTime = xTaskGetTickCount();

	system_packet system_buffer = {0};
	while(1)
	{
    if(xQueueReceive(system_queue, (void * )&system_buffer, 2))
    {
      ESP_LOGI(TAG, "Time since boot: %lld us", esp_timer_get_time());
      
      ESP_LOGI(TAG, "[Target State]: %d", system_buffer.data[0]);
      ESP_LOGI(TAG, "[Moving Target]: %d", system_buffer.data[1]);
      ESP_LOGI(TAG, "[Stationary Target]: %d", system_buffer.data[3]);
      // Displays received target distance data
      // display(system_buffer.data[1]);
    }

    // Delays the task execution until a specific time
    // This ensures the task wakes up periodically at least every 10 milliseconds
    // vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10) );
	}
}

void app_main(void)
{
  uart_config();

  xTaskCreate(uart_event_task, "uart_event_task", 10000, NULL, 4, NULL);
  xTaskCreate(uart_reception_task, "uart_rx_task", 10000, NULL, 4, NULL);
  // xTaskCreate(uart_transmission_task, "uart_tx_task", 1024*2, NULL, 4, NULL);
  xTaskCreate(display_time_task, "display_time_task", 10000, NULL, 4, NULL);

  // memset(fw_version, 0, sizeof(fw_version));
  
  // if (!(read_firmware_version(handle, fw_version))) {
  //   ESP_LOGE(TAG, "Error getting FW Version message:");

  //   // ESP_LOGE(TAG, "Error getting FW Version message: 0x%x", err);
  // }

  // // Radar ACK (success)
  // // Command word: 2 bytes For example 0x01A0
  // // Return value: 2-bytes ACK status (0 successful, 1 failed) + 2-bytes firmware type (0x2412)+2-bytes major
  // //                version number+4-bytes minor version number
  // ESP_LOGI(TAG, "LD2412 firmware type: %02X.%02X", fw_version[5], fw_version[4]);
  // ESP_LOGI(TAG, "LD2412 firmware version: V%02X.%02X.%02X.%02X.%02X.%02X", fw_version[7], fw_version[6], fw_version[11], fw_version[10], fw_version[9], fw_version[8]);
  
}