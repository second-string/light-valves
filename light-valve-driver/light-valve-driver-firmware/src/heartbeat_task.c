#include "heartbeat_task.h"

static void gpio_pin_t *heartbeat_pin;

static void heartbeat_task(void *args) {
    while (1) {
        gpio_toggle_pin(pin);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void heartbeat_task_init(gpio_pin_t *pin) {
    heartbeat_pin = pin;
}

void heartbeat_task_start() {
    BaseType_t rval =
        xTaskCreate(heartbeat_task, "heartbeat", configMINIMALSTACKSIZE * 3, NULL, tskIDLE_PRIORITY, NULL);
    configASSERT(rval);
}
