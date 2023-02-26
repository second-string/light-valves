#include "FreeRTOS.h"
#include "task.h"

#include "output_data_task.h"

// Period of single clock cycle for data transmission
#define CLOCK_PERIOD_US (50)

// MUST MATCH HW TIMER INTERRUPT CONFIGURATION. The cadence at which the hw timer ISR fires - easier to make > 1us to
// ease context switching burder since the only two times we need to delay for are CLOCK_PERIOD_US and CLOCK_PERIOD_US
// / 2.
#define TIMER_TICK_PERIOD_US (5)

#define ZERO_BIT_CLOCK_TICKS ((CLOCK_PERIOD_US) / 2 / (TIMER_TICK_PERIOD_US))
#define ONE_BIT_CLOCK_TICKS ((CLOCK_PERIOD_US) / (TIMER_TICK_PERIOD_US))

// Store singleton handle globally for access to members
static output_data_task_handle_t *handle;

// Keep these outside the task handle for the minor perf benefit of one less address redirect
static volatile bool     waiting;
static volatile uint32_t current_delay_ticks;

static gpio_pin_t *tx_dbg_pin;

/*
 * Sets the proper delay time and flag to be checked by the tick handler executed from the HW timer ISR. Spinlocks until
 * ISR-called function re-toggles data GPIO pin and sets flag to false (all RTOS constructs too slow). GPIO toggled from
 * tick handler to keep timing as tight as possible to clock period instead of having to wait for another context
 * switch.
 */
static inline void delay_ticks_and_toggle(uint32_t ticks_to_delay) {
    current_delay_ticks = ticks_to_delay - 2;
    waiting             = true;

    while (waiting) {
        ;
    }
}

static inline void send_bit(uint8_t bit) {
    configASSERT(bit == 0 || bit == 1);

    if (bit) {
        delay_ticks_and_toggle(ONE_BIT_CLOCK_TICKS);
    } else {
        delay_ticks_and_toggle(ZERO_BIT_CLOCK_TICKS);
        delay_ticks_and_toggle(ZERO_BIT_CLOCK_TICKS);
    }
}

static void send_node_packet(uint8_t addr_nibble, uint8_t data_nibble) {
    // Initial transition
    gpio_toggle_pin(handle->output_pin);

    // send clock rate
    send_bit(1);

    // waiting for start state between clock rate and preamble bits
    send_bit(1);

    // start bits
    send_bit(1);
    send_bit(0);
    send_bit(1);
    send_bit(0);

    // address bits
    for (int i = 0; i < 4; i++) {
        send_bit((addr_nibble >> (3 - i)) & 0x1);
    }

    // address bits
    for (int i = 0; i < 4; i++) {
        send_bit((data_nibble >> (3 - i)) & 0x1);
    }
}

static void output_data_task(void *args) {
    // Enable UPDATE interrupt to trigger on timer period expiration and enable timer
    TIM1->DIER |= TIM_DIER_UIE;
    TIM1->CR1 |= TIM_CR1_CEN;

    uint16_t   node_data     = 0x0;
    uint8_t   *node_data_ptr = (uint8_t *)&node_data;
    uint8_t    address       = 0x0;
    uint8_t    data          = 0x0;
    BaseType_t rval          = pdFALSE;
    while (1) {
        rval = xQueueReceive(handle->data_out_queue, &node_data, portMAX_DELAY);
        configASSERT(rval);

        // TODO : enable timer and disable after sending to help w/ context switching?
        // Each byte in the node_data uint16_t received from the queue is 4 bits node addr and 4 bits node data. Iterate
        // through each byte in uint16_t value, cast and shift each nibble if needed, and then send packet with those
        // values
        for (uint8_t i = 0; i < 16; i++) {
            address = (node_data_ptr[i] & 0xF0) >> 4;
            data    = node_data_ptr[i] & 0x0F;
            send_node_packet(address, data);

            gpio_toggle_pin(tx_dbg_pin);

            // We really only need ~1ms between node packets for its timeout to trigger so it knows it's time to parse a
            // new packet. vTaskDelaying with arg of 1 results in actual delay of ~300us, so bump the arg to 2ms to get
            // ~1.4ms delay. Can be tuned in conjunection w/ node timeout timer if we want less time between node
            // packets.
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

// Assumes length of node_data matches 16 bytes queue created with for item size
bool output_data_task_send_packet(uint8_t *node_data) {
    return xQueueSendToBack(handle->data_out_queue, node_data, portMAX_DELAY);
}

/*
 * Data output "clock" tick function called every time the HW timer interrupts. Does nothing if not currently waiting to
 * send a bit, otherwise compares remaining ticks and clears flag that delay_ticks_and_toggle is waiting on.
 */
void output_data_task_clock_tick() {
    if (waiting) {
        if (current_delay_ticks == 0) {
            gpio_toggle_pin(handle->output_pin);
            waiting = false;
        } else {
            current_delay_ticks -= 1;
        }
    }
}

void output_data_task_init(gpio_pin_t                *pin,
                           gpio_pin_t                *tx_led_pin,
                           output_data_task_handle_t *output_data_task_handle) {
    configASSERT(pin);
    configASSERT(tx_led_pin);

    handle                 = output_data_task_handle;
    handle->output_pin     = pin;
    handle->data_out_queue = xQueueCreate(5, 16 * sizeof(uint8_t));

    current_delay_ticks = 0;
    waiting             = false;

    tx_dbg_pin = tx_led_pin;
}

void output_data_task_start() {
    configASSERT(handle);
    (void)output_data_task;
    BaseType_t rval = xTaskCreate(output_data_task, "output data task", configMINIMAL_STACK_SIZE * 4, NULL, 6, NULL);
    configASSERT(rval == pdTRUE);
}
