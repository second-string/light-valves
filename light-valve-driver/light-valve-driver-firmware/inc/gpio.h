#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "stm32f1xx_ll_gpio.h"

typedef enum {
    GPIO_PIN_DIR_OUTPUT,
    GPIO_PIN_DIR_INPUT,
} gpio_pin_dir_t;

typedef struct {
    GPIO_TypeDef  *port;
    uint32_t       mask;
    gpio_pin_dir_t dir;
} gpio_pin_t;

bool gpio_read_pin(gpio_pin_t *pin);
void gpio_set_pin(gpio_pin_t *pin);
void gpio_clear_pin(gpio_pin_t *pin);
void gpio_toggle_pin(gpio_pin_t *pin);
