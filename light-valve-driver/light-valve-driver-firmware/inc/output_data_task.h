#pragma once

#include "FreeRTOS.h"
#include "queue.h"

#include "gpio.h"

typedef struct {
    gpio_pin_t   *output_pin;
    QueueHandle_t data_out_queue;
} output_data_task_handle_t;

typedef struct __attribute__((packed)) {
    union {
        uint16_t raw;
        struct __attribute__((packed)) {
            uint16_t clock_period : 1;
            uint16_t ready_bit : 1;
            uint16_t start_bits : 4;
            uint16_t address : 4;
            uint16_t data : 4;
        };
    };
} data_output_packet_t;

bool output_data_task_send_packet(data_output_packet_t *packet);
void output_data_task_clock_tick();
void output_data_task_init(gpio_pin_t *pin, output_data_task_handle_t *handle);
void output_data_task_start();
