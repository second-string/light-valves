#include "FreeRTOS.h"
#include "task.h"

#include "gpio.h"
#include "input_data_task.h"
#include "output_data_task.h"

input_data_task_handle_t *handle;

void dbg_print_char(char c) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&c, 1, portMAX_DELAY);
}

static void input_data_task(void *args) {
    configASSERT(handle);

    // uint8_t              c;
    data_output_packet_t packet = {
        .address = 0x1,
        .data    = 0x0,
    };
    int8_t mult = 1;
    while (1) {
        // Blocking forever for now
        // HAL_UART_Receive(handle->uart_handle->uart, &c, sizeof(uint8_t), portMAX_DELAY);
        // if (handle->uart_handle->process_char) {
        //     handle->uart_handle->process_char(c);
        // } else {
        //     configASSERT(0);
        // }

        output_data_task_send_packet(&packet);
        packet.data += mult;
        if (packet.data == 0) {
            mult = 1;
        } else if (packet.data == 15) {
            mult = -1;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void input_data_task_init(uart_handle_t *uart_handle, input_data_task_handle_t *input_data_task_handle) {
    configASSERT(uart_handle);

    handle                            = input_data_task_handle;
    handle->uart_handle               = uart_handle;
    handle->uart_handle->process_char = dbg_print_char;
}

void input_data_task_start(input_data_task_handle_t *handle) {
    configASSERT(handle);
    BaseType_t rval =
        xTaskCreate(input_data_task, "input data task", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY, NULL);
    configASSERT(rval == pdTRUE);
}
