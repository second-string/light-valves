#pragma once

#include "uart.h"

typedef struct {
    uart_handle_t *uart_handle;
} input_data_task_handle_t;

void input_data_task_init(uart_handle_t *uart_handle, input_data_task_handle_t *handle);
void input_data_task_start(input_data_task_handle_t *handle);
