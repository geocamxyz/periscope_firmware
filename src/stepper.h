//
// Created by linus on 6/26/25.
//

#ifndef STEPPER_H
#define STEPPER_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define STEPPER_THREAD_STACK_SIZE 1024
#define STEPPER_THREAD_PRIORITY 3

extern struct k_thread motor_thread_data;

// Function declarations
void stepper_thread_entry(void *, void *, void *);
void stepper_control_start(void);

#endif //STEPPER_H
