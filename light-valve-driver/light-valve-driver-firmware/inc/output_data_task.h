#pragma once

#include "FreeRTOS.h"
#include "queue.h"

#include "gpio.h"

typedef struct {
    gpio_pin_t   *output_pin;
    QueueHandle_t data_out_queue;
} output_data_task_handle_t;

// typedef struct __attribute__((packed)) {
//     union {
//         uint16_t raw;
//         struct __attribute__((packed)) {
//             uint16_t clock_period : 1;
//             uint16_t ready_bit : 1;
//             uint16_t start_bits : 4;
//             uint16_t address : 4;
//             uint16_t data : 4;
//         };
//     };
// } output_data_packet_t;

bool output_data_task_send_packet(uint8_t *node_data);
void output_data_task_clock_tick();
void output_data_task_init(gpio_pin_t *pin, gpio_pin_t *tx_led_pin, output_data_task_handle_t *handle);
void output_data_task_start();
