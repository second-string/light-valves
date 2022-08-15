#include "FreeRTOS.h"
#include "task.h"

#include "heartbeat_task.h"

static gpio_pin_t *heartbeat_pin;

static void heartbeat_task(void *args) {
    while (1) {
        gpio_toggle_pin(heartbeat_pin);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void heartbeat_task_init(gpio_pin_t *pin) {
    heartbeat_pin = pin;
}

void heartbeat_task_start() {
    BaseType_t rval =
        xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE * 3, NULL, /*tskIDLE_PRIORITY*/ 0, NULL);
    configASSERT(rval);
}
