// Pull the pin name definitions from freertos main.h
#include "main.h"

#include "FreeRTOS.h"

#include "gpio.h"
#include "heartbeat_task.h"

gpio_pin_t devboard_led_pin = {
    .port = DEVBOARD_LED_GPIO_Port,
    .mask = DEVBOARD_LED_Pin,
    .dir  = GPIO_PIN_DIR_OUTPUT,
};

gpio_pin_t led1_pin = {
    .port = LED1_IO_GPIO_Port,
    .mask = LED1_IO_Pin,
    .dir  = GPIO_PIN_DIR_OUTPUT,
};

gpio_pin_t led2_pin = {
    .port = LED2_IO_GPIO_Port,
    .mask = LED2_IO_Pin,
    .dir  = GPIO_PIN_DIR_OUTPUT,
};

gpio_pin_t dbg_tx_pin = {
    .port = DBG_TX_GPIO_Port,
    .mask = DBG_TX_Pin,
    .dir  = GPIO_PIN_DIR_OUTPUT,
};

gpio_pin_t dbg_rx_pin = {
    .port = DBG_RX_GPIO_Port,
    .mask = DBG_RX_Pin,
    .dir  = GPIO_PIN_DIR_INPUT,
};

gpio_pin_t data_pin = {
    .port = DATA_GPIO_Port,
    .mask = DATA_Pin,
    .dir  = GPIO_PIN_DIR_OUTPUT,
};

gpio_pin_t xcvr_en_pin = {
    .port = XCVR_EN_GPIO_Port,
    .mask = XCVR_EN_Pin,
    .dir  = GPIO_PIN_DIR_OUTPUT,
};

gpio_pin_t xcvr_di_pin = {
    .port = XCVR_DI_GPIO_Port,
    .mask = XCVR_DI_Pin,
    .dir  = GPIO_PIN_DIR_OUTPUT,
};

gpio_pin_t xcvr_ro_pin = {
    .port = XCVR_RO_GPIO_Port,
    .mask = XCVR_RO_Pin,
    .dir  = GPIO_PIN_DIR_INPUT,
};

void app_init(void) {
    heartbeat_task_init(&devboard_led_pin);
}

void app_start(void) {
    heartbeat_task_start();
}

void app_main(void) {
    app_init();
    app_start();

    while (1) {
    }
}
