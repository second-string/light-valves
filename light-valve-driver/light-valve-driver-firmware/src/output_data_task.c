#include "FreeRTOS.h"
#include "task.h"

#include "output_data_task.h"

#define CLOCK_PERIOD_US (50)
#define TIMER_TICK_PERIOD_US (5)
#define ZERO_BIT_CLOCK_TICKS ((CLOCK_PERIOD_US) / 2 / (TIMER_TICK_PERIOD_US))
#define ONE_BIT_CLOCK_TICKS ((CLOCK_PERIOD_US) / (TIMER_TICK_PERIOD_US))

// Used for task notifications when we reach the desired clock ticks so we don't have to spin
static TaskHandle_t task_handle;

static volatile bool     waiting;
static volatile uint32_t current_delay_ticks;

static inline void delay_ticks_and_toggle(uint32_t ticks_to_delay) {
    current_delay_ticks = ticks_to_delay - 2;
    waiting             = true;

    // Spinlock until ISR-called function re-toggles data pin and sets flag to false. All RTOS constructs too slow
    while (waiting) {
        ;
    }
}

static inline void send_bit(uint8_t bit, output_data_task_handle_t *handle) {
    configASSERT(bit == 0 || bit == 1);

    if (bit) {
        delay_ticks_and_toggle(ONE_BIT_CLOCK_TICKS);
    } else {
        delay_ticks_and_toggle(ZERO_BIT_CLOCK_TICKS);
        delay_ticks_and_toggle(ZERO_BIT_CLOCK_TICKS);
    }
}

static void send_test_packet(uint8_t addr_nibble, uint8_t data_nibble, output_data_task_handle_t *handle) {
    // Initial transition
    gpio_toggle_pin(handle->output_pin);

    // send clock rate
    send_bit(1, handle);

    // waiting for start state between clock rate and preamble bits
    send_bit(1, handle);

    // start bits
    send_bit(1, handle);
    send_bit(0, handle);
    send_bit(1, handle);
    send_bit(0, handle);

    // address bits
    for (int i = 0; i < 4; i++) {
        send_bit((addr_nibble >> (3 - i)) & 0x1, handle);
    }

    // address bits
    for (int i = 0; i < 4; i++) {
        send_bit((data_nibble >> (3 - i)) & 0x1, handle);
    }
}

static void output_data_task(void *args) {
    output_data_task_handle_t *handle = args;
    configASSERT(handle);

    // Enable UPDATE interrupt to trigger on timer period expiration and enable timer
    TIM1->DIER |= TIM_DIER_UIE;
    TIM1->CR1 |= TIM_CR1_CEN;

    uint8_t i = 0;
    while (1) {
        // Send 32 packets w/ 2ms between each, wait 5 seconds, repeat
        uint8_t addr = 0x1;
        send_test_packet(addr, i, handle);
        i++;
        if (i == 15) {
            i = 0;
        }

        /*
        for (int i = 0; i < 32; i++) {
            if (i % 2 == 0) {
                addr = 0x1;
            } else {
                addr = 0x2;
            }

            send_test_packet(addr, i, handle);
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        */

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void inline output_data_task_clock_tick() {
    if (waiting) {
        if (current_delay_ticks == 0) {
            LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_0);
            waiting = false;
        } else {
            current_delay_ticks -= 1;
        }
    }
}

void output_data_task_init(gpio_pin_t *pin, output_data_task_handle_t *handle) {
    configASSERT(pin);

    handle->output_pin  = pin;
    current_delay_ticks = 0;
    // clock_ticks        = 0;
    // start_clock_ticks  = 0;
    waiting = false;
}

void output_data_task_start(output_data_task_handle_t *handle) {
    configASSERT(handle);
    BaseType_t rval =
        xTaskCreate(output_data_task, "output data task", configMINIMAL_STACK_SIZE * 4, handle, 6, &task_handle);
    configASSERT(rval == pdTRUE);
}
