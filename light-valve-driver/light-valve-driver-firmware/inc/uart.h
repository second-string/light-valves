#pragma once

#include "FreeRTOS.h"
#include "queue.h"

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_usart.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

typedef void (*process_char_func)(char c);

typedef struct {
    // uart_port_t       port;
    // uart_config_t     config;
    QueueHandle_t       queue;
    char               *rx_buffer;
    process_char_func   process_char;
    UART_HandleTypeDef *uart;
} uart_handle_t;

// Build out handle properties
void uart_init(UART_HandleTypeDef *uart, process_char_func process_char_cb, uart_handle_t *handle);
void uart_generic_rx_task(void *args);
