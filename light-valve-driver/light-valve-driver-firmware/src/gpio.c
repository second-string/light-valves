#include "gpio.h"

bool gpio_read_pin(gpio_pin_t *pin) {
    uint32_t full_port = 0;
    if (pin->dir == GPIO_PIN_DIR_OUTPUT) {
        full_port = LL_GPIO_ReadOutputPort(pin->port);
    } else {
        full_port = LL_GPIO_ReadInputPort(pin->port);
    }

    return full_port & (1 << pin->mask);
}

void gpio_set_pin(gpio_pin_t *pin) {
    LL_GPIO_SetOutputPin(pin->port, pin->mask);
}

void gpio_clear_pin(gpio_pin_t *pin) {
    LL_GPIO_ResetOutputPin(pin->port, pin->mask);
}

void gpio_toggle_pin(gpio_pin_t *pin) {
    LL_GPIO_TogglePin(pin->port, pin->mask);
}
