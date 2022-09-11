#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "uart.h"

/*
 * Most init functionality done by cubemx in generated MX init in main.c
 */
void uart_init(UART_HandleTypeDef *uart, process_char_func process_char_cb, uart_handle_t *handle) {
    configASSERT(uart);

    handle->process_char = process_char_cb;
    handle->uart         = uart;
}

void uart_generic_rx_task(void *args) {
    uart_handle_t *handle = args;
    configASSERT(handle);

    uint8_t c;
    while (1) {
        // Blocking forever for now
        HAL_UART_Receive(handle->uart, &c, sizeof(uint8_t), portMAX_DELAY);
        if (handle->process_char) {
            handle->process_char(c);
        } else {
            char  *base     = "uart_generic_rx_task RX byte, no process_byte handler:";
            size_t base_len = strlen(base);
            char   full[base_len + 3];
            sprintf(full, "%s %c", base, c);
            HAL_UART_Transmit(handle->uart, (uint8_t *)full, strlen(full), portMAX_DELAY);
        }
    }
}
