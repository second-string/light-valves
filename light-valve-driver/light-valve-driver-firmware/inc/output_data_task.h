#pragma once

#include "gpio.h"

typedef struct {
    gpio_pin_t *output_pin;
} output_data_task_handle_t;

void output_data_task_init(gpio_pin_t *pin, output_data_task_handle_t *handle);
void output_data_task_start(output_data_task_handle_t *handle);
