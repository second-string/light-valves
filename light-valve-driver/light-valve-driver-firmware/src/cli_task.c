#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "FreeRTOS_CLI.h"
#include "cli_task.h"
#include "constants.h"
#include "uart.h"

// Lower number, lower priority (idle == 0)
#define CLI_TASK_PRIORITY (tskIDLE_PRIORITY)
#define CLI_COMMAND_INPUT_BUFFER_BYTES (128)
#define CLI_COMMAND_DMA_OUTPUT_BUFFER_BYTES (1024)

#define CLI_CMD_PROCESS_TASK_PRIORITY (tskIDLE_PRIORITY)
#define CLI_COMMAND_QUEUE_SIZE (12)
#define CLI_COMMAND_PROCESS_OUT_BUFFER_BYTES (128)

typedef struct {
    char  *cmd;
    size_t len;
} cli_command_t;

static uart_handle_t *handle;
static char          *command_buffer;
static uint8_t        command_char_idx;
static char          *dma_output_buffer;

static QueueHandle_t queue_handle;
static StaticQueue_t queue_buffer;
static uint8_t      *queue_data_buffer;
static char         *command_processing_out;

const char *backspace = "\x08\x20\x08";
const char *newline   = "\r\n";

static void cli_process_char(char c) {
    // Handle BS and DEL ascii chars
    if (c == 0x08 || c == 0x7F) {
        command_buffer[--command_char_idx] = 0x00;
        // Echo out a backspace to move the cursor back, a space to cover up the old char, then another backspace to
        // remove the space. Don't put any of it in our buffer because we dgaf about those shenanigans to make it look
        // good to the user.
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)backspace, 3);
    } else {
        // Add to buffer and echo back
        command_buffer[command_char_idx++] = c;
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&c, 1);
    }

    if (c == '\n' || c == '\r') {
        // Overwrite newline with null term FreeRTOS+CLI to be able to process it
        // This is assuming that we're only getting a single CR or LF - if we get two this will break command parsing
        command_char_idx--;
        command_buffer[command_char_idx] = 0x00;

        // This uses blocking DMA because I'm lazy. When we rx a command, we'll be outputting the command response
        // almost immediately after this, so that call to tx dma will overwrite the newline in the DMA buffer. Instead
        // of tracking the DMA pointer or waiting on uart tx status, we just blocking tx this so we know that the uart
        // is ready when we go to output cmd response.
        HAL_UART_Transmit(&huart2, (uint8_t *)newline, 2, portMAX_DELAY);

        // Either free on failure here or dequeuer responsible for freeing
        char *cmd_copy = pvPortMalloc((command_char_idx + 1) * sizeof(char));
        memcpy(cmd_copy, command_buffer, command_char_idx + 1);
        cli_command_t cmd = {
            .cmd = cmd_copy,
            .len = command_char_idx * sizeof(char),
        };

        // Queue will copy cmd struct values to internal queue items, no need to malloc it
        BaseType_t rval = xQueueSendToBack(queue_handle, &cmd, pdMS_TO_TICKS(10));
        if (!rval) {
            vPortFree(cmd_copy);
        }

        command_char_idx = 0;
    }
}

/*
 * Task to pop commands off the queue and execute them. Seperates long or blocking command handlers from serial rx
 */
static void cli_process_command(void *args) {
    QueueHandle_t queue = args;

    BaseType_t    rval                  = pdFALSE;
    BaseType_t    more_data             = pdFALSE;
    uint16_t      total_bytes_written   = 0;
    uint16_t      bytes_received        = 0;
    uint8_t       current_bytes_written = 0;
    cli_command_t cmd;
    while (1) {
        rval = xQueueReceive(queue, &cmd, portMAX_DELAY);
        configASSERT(rval);

        total_bytes_written   = 0;
        bytes_received        = 0;
        current_bytes_written = 0;
        memset(dma_output_buffer, 0, CLI_COMMAND_PROCESS_OUT_BUFFER_BYTES);
        do {
            // Pass cmd to cli and receive string to output + indication if more output strings coming
            more_data =
                FreeRTOS_CLIProcessCommand(cmd.cmd, command_processing_out, CLI_COMMAND_PROCESS_OUT_BUFFER_BYTES);
            bytes_received = strlen(command_processing_out);

            // Copy string to output into output buffer only up to end of buffer size (leaving room for newline + nul
            // term)
            current_bytes_written = MIN(bytes_received, CLI_COMMAND_DMA_OUTPUT_BUFFER_BYTES - 3 - total_bytes_written);
            memcpy(dma_output_buffer + total_bytes_written, command_processing_out, current_bytes_written);
            total_bytes_written += current_bytes_written;
            strcpy(dma_output_buffer + total_bytes_written, newline);

            // Only increment total by the newline, not the null term copied by strlen. If more_data == true, we want
            // the next line of output data to overwrite the null term
            total_bytes_written += 2;
        } while (more_data && total_bytes_written < (CLI_COMMAND_DMA_OUTPUT_BUFFER_BYTES - 1));

        // Pass off finished buffer to DMA
        total_bytes_written += 1;
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)dma_output_buffer, total_bytes_written);

        // Free the string malloced by uart rx task
        vPortFree(cmd.cmd);
    }
}

void cli_task_init(uart_handle_t *uart_handle) {
    configASSERT(uart_handle);
    handle = uart_handle;

    // Weird situation where this uart is already fully inited since we need to do it first thing to log out init
    // sequence. For other uart users, they'd probably init the entire uart_handle_t with uart_init themselves.
    // Instead we just assign the callback here and drop any received chars that happen in init sequence
    handle->process_char = cli_process_char;

    command_buffer = pvPortMalloc(CLI_COMMAND_INPUT_BUFFER_BYTES * sizeof(char));
    configASSERT(command_buffer);
    dma_output_buffer = pvPortMalloc(CLI_COMMAND_DMA_OUTPUT_BUFFER_BYTES * sizeof(char));
    configASSERT(dma_output_buffer);
    command_char_idx = 0;

    queue_data_buffer = pvPortMalloc(CLI_COMMAND_QUEUE_SIZE * sizeof(cli_command_t));
    configASSERT(queue_data_buffer);
    queue_handle = xQueueCreateStatic(CLI_COMMAND_QUEUE_SIZE, sizeof(cli_command_t), queue_data_buffer, &queue_buffer);
    configASSERT(queue_handle);
    command_processing_out = pvPortMalloc(CLI_COMMAND_PROCESS_OUT_BUFFER_BYTES * sizeof(char));
    configASSERT(command_processing_out);
}

void cli_task_start() {
    BaseType_t rval =
        xTaskCreate(uart_generic_rx_task, "CLI UART RX", configMINIMAL_STACK_SIZE * 4, handle, CLI_TASK_PRIORITY, NULL);
    configASSERT(rval == pdTRUE);

    rval = xTaskCreate(cli_process_command,
                       "CLI cmd process",
                       configMINIMAL_STACK_SIZE * 3,
                       queue_handle,
                       CLI_CMD_PROCESS_TASK_PRIORITY,
                       NULL);
    configASSERT(rval == pdTRUE);
}
