#include <string.h>

// Pull the pin name definitions from freertos main.h
#include "main.h"

#include "stm32f1xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "cli_commands.h"
#include "cli_task.h"
#include "gpio.h"
#include "heartbeat_task.h"
#include "input_data_task.h"
#include "output_data_task.h"
#include "uart.h"

// Individual task data handles
static uart_handle_t             cli_uart_handle;
static uart_handle_t             xcvr_uart_handle;
static input_data_task_handle_t  input_data_task_handle;
static output_data_task_handle_t output_data_task_handle;

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
    output_data_task_init(&data_pin, &output_data_task_handle);

    uart_init(&huart1, 0, 0, 0, 0, NULL, &xcvr_uart_handle);
    input_data_task_init(&xcvr_uart_handle, 0x01, &input_data_task_handle);

    // NULL passed for process_char callback, see cli_task_init for reasoning
    uart_init(&huart2,
              CLI_UART_RX_RING_BUFFER_BYTES,
              CLI_UART_TX_RING_BUFFER_BYTES,
              CLI_UART_QUEUE_SIZE,
              CLI_UART_RX_BUFFER_BYTES,
              NULL,
              &cli_uart_handle);
    cli_task_init(&cli_uart_handle);
    cli_command_register_all();

    heartbeat_task_init(&devboard_led_pin);
}

void app_start(void) {
    input_data_task_start();
    output_data_task_start();
    cli_task_start();
    heartbeat_task_start();
}

void app_main(void) {
    app_init();
    app_start();

    size_t info_buffer_size = 200 * sizeof(char);
    char   info_buffer[info_buffer_size];
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, portMAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, portMAX_DELAY);
    while (cli_command_info(info_buffer, info_buffer_size, NULL) == pdTRUE) {
        HAL_UART_Transmit(&huart2, (uint8_t *)info_buffer, strlen(info_buffer), portMAX_DELAY);
        HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, portMAX_DELAY);
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, portMAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, portMAX_DELAY);
}
