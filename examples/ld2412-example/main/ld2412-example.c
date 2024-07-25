#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include <esp_timer.h>

#include "ld2412.h"
#include "uart_config.h"
#include "button_config.h"

static const char *TAG = "LD2412DEMO";

/* VARIABLES -----------------------------------------------------------------*/
QueueHandle_t system_queue;

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
      // ESP_LOGI(TAG, "Time since boot: %lld us", esp_timer_get_time());
      ESP_LOGI(TAG, "[Target State]: %d", system_buffer.data[0]);
      ESP_LOGI(TAG, "[Moving Target]: %d", system_buffer.data[1]);
      ESP_LOGI(TAG, "[Stationary Target]: %d", system_buffer.data[2]);

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
  button_init(BOOT_BUTTON_NUM);
  
  xTaskCreate(uart_event_task, "uart_event_task", 10000, NULL, 4, NULL);
  xTaskCreate(uart_transmission_task, "uart_tx_task", 10000, NULL, 4, NULL);
  xTaskCreate(uart_reception_task, "uart_rx_task", 10000, NULL, 4, NULL);
  xTaskCreate(display_time_task, "display_time_task", 10000, NULL, 4, NULL);
}