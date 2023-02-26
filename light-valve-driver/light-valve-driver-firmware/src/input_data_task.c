#include "FreeRTOS.h"
#include "task.h"

#include "input_data_task.h"
#include "output_data_task.h"

static gpio_pin_t *rx_dbg_pin;

// Assumes all 16 node_data slots are always filled for now, would be good to have a 4-bit field for num node_data slots
// used for more efficient comms
typedef struct __attribute__((packed)) {
    uint8_t start_bits : 4;
    uint8_t addr_bits : 4;
    uint8_t node_data[16];
} input_data_packet_t;

static input_data_task_handle_t *handle;
static TaskHandle_t              task_handle;
static input_data_packet_t       input_packet;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // Disable IDLE interrupt until after we process packet since we got a full packet here and don't need to timeout
    // and reset DMA
    __HAL_UART_DISABLE_IT(handle->uart_handle->uart, UART_IT_IDLE);

    BaseType_t yield = pdFALSE;
    // TODO :: make this a queue instead of forcing task to process immediately each packet received
    vTaskNotifyGiveFromISR(task_handle, &yield);
    portYIELD_FROM_ISR(yield);
}

/*
 * Called from stm32f1xx_it.c UART ISR when IDLE interrupt fires for incoming xcvr UART line. Handles byte drops by
 * resetting DMA if we get an idle interrupt and we didn't also get an RxCplt interrupt right before this.
 */
void input_data_task_uart_idle() {
    HAL_UART_DMAStop(handle->uart_handle->uart);
    HAL_UART_Receive_DMA(handle->uart_handle->uart, (uint8_t *)&input_packet, sizeof(input_data_packet_t));
}

static void input_data_task(void *args) {
    configASSERT(handle);

    uint32_t count = 0;

    __HAL_UART_ENABLE_IT(handle->uart_handle->uart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(handle->uart_handle->uart, (uint8_t *)&input_packet, sizeof(input_data_packet_t));

    while (1) {
        count = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        (void)count;

        // If packet from rs485 has matching start and address bits for this driver, enqueue full 16-element
        // node_data array for output task to process and send. Start bits should be 0b0110.
        if (input_packet.start_bits == 0x6 && input_packet.addr_bits == handle->device_address) {
            output_data_task_send_packet(input_packet.node_data);
            gpio_toggle_pin(rx_dbg_pin);
        }

        // Clear IDLE flag if it was somehow pending and re-enable interrupt to catch incomplete packets
        // This enable/disable logic seems to work when running uart at 115200+, but not 9600. I think 9600 is so slow
        // that the processing code executes and the interrupt is enabled again before one more full uart frame passes
        // which causes the interrupt to fire.
        __HAL_USART_CLEAR_IDLEFLAG(&huart1);
        __HAL_UART_ENABLE_IT(handle->uart_handle->uart, UART_IT_IDLE);
    }
}

void input_data_task_init(uart_handle_t            *uart_handle,
                          uint16_t                  device_address,
                          gpio_pin_t               *rx_led_pin,
                          input_data_task_handle_t *input_data_task_handle) {
    configASSERT(uart_handle);
    configASSERT(rx_led_pin);

    handle                 = input_data_task_handle;
    handle->uart_handle    = uart_handle;
    handle->device_address = device_address;

    rx_dbg_pin = rx_led_pin;
}

void input_data_task_start(input_data_task_handle_t *handle) {
    configASSERT(handle);
    BaseType_t rval = xTaskCreate(input_data_task,
                                  "input data task",
                                  configMINIMAL_STACK_SIZE * 3,
                                  NULL,
                                  tskIDLE_PRIORITY,
                                  &task_handle);
    configASSERT(rval == pdTRUE);
}
