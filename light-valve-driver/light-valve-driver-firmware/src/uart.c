#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "uart.h"

UART_HandleTypeDef *cli_uart;

/*
 * Most init functionality done by cubemx in generated MX init in main.c
 */
void uart_init(UART_HandleTypeDef *uart,
               uint16_t            rx_ring_buffer_size,
               uint16_t            tx_ring_buffer_size,
               uint8_t             event_queue_size,
               uint16_t            rx_buffer_size,
               process_char_func   process_char_cb,
               uart_handle_t      *handle) {
    configASSERT(uart);
    cli_uart = uart;

    // handle->rx_buffer = pvPortMalloc(sizeof(char) * rx_buffer_size);
    // configASSERT(handle->rx_buffer);

    handle->process_char = process_char_cb;
}

void uart_generic_rx_task(void *args) {
    uart_handle_t *handle = args;
    configASSERT(handle);

    uint8_t c;
    while (1) {
        // Underlying impl uses esp-freertos xRingBufferReceive which I'm pretty sure used a freertos primitive below
        // it, so this portMAX_DELAY should do our regular yield like we want instead of spinning
        // const int bytes_read = uart_read_bytes(handle->port, handle->rx_buffer, 1, portMAX_DELAY);
        HAL_UART_Receive(cli_uart, &c, sizeof(uint8_t), portMAX_DELAY);
        if (handle->process_char) {
            handle->process_char(c);
        } else {
            char  *base     = "uart_generic_rx_task RX byte, no process_byte handler:";
            size_t base_len = strlen(base);
            char   full[base_len + 3];
            sprintf(full, "%s %c", base, c);
            HAL_UART_Transmit(cli_uart, (uint8_t *)full, strlen(full), portMAX_DELAY);
        }
    }
}
