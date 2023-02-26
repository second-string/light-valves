#pragma once

#include <stdbool.h>

#include "gpio.h"
#include "uart.h"

typedef struct {
    uart_handle_t *uart_handle;
    uint16_t       device_address;
} input_data_task_handle_t;

void input_data_task_uart_update();
void input_data_task_uart_idle();
void input_data_task_init(uart_handle_t            *uart_handle,
                          uint16_t                  device_address,
                          gpio_pin_t               *rx_led_pin,
                          input_data_task_handle_t *input_data_task_handle);
void input_data_task_start();
