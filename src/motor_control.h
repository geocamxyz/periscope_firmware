#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define MOTOR_THREAD_STACK_SIZE 1024
#define MOTOR_THREAD_PRIORITY 6
#define MOTOR_UART_NODE DT_ALIAS(motor_uart)

// Declare as extern - definitions will be in motor_control.c
extern const struct gpio_dt_spec motor_index;
extern volatile uint32_t motor_step_counter;
extern struct gpio_callback step_cb_data;
extern const struct device *const motor_uart;
extern struct k_thread motor_thread_data;

// Function declarations
void motor_thread_entry(void *, void *, void *);
void motor_control_start(void);

#endif