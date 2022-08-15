#pragma once

#include "gpio.h"

void heartbeat_task_init(gpio_pin_t *pin);
void heartbeat_task_start();
