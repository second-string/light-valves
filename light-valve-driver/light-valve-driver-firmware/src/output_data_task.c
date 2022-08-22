#include "FreeRTOS.h"
#include "task.h"

#include "output_data_task.h"

static void output_data_task(void *args) {
    output_data_task_handle_t *handle = args;
    configASSERT(handle);

    while (1) {
        gpio_toggle_pin(handle->output_pin);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void output_data_task_init(gpio_pin_t *pin, output_data_task_handle_t *handle) {
    configASSERT(pin);

    handle->output_pin = pin;
}

void output_data_task_start(output_data_task_handle_t *handle) {
    configASSERT(handle);
    BaseType_t rval =
        xTaskCreate(output_data_task, "output data task", configMINIMAL_STACK_SIZE * 3, handle, tskIDLE_PRIORITY, NULL);
    configASSERT(rval == pdTRUE);
}
