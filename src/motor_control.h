#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <zephyr/kernel.h>

#define MOTOR_THREAD_STACK_SIZE 1024
#define MOTOR_THREAD_PRIORITY 6
#define MOTOR_UART_NODE DT_ALIAS(motor_uart)

static const struct gpio_dt_spec motor_index = GPIO_DT_SPEC_GET(DT_NODELABEL(motor_index), gpios);
static volatile uint32_t motor_step_counter = 0;
static struct gpio_callback step_cb_data;

static const struct device *const motor_uart = DEVICE_DT_GET(MOTOR_UART_NODE);

K_THREAD_STACK_DEFINE(motor_thread_stack, MOTOR_THREAD_STACK_SIZE);
static struct k_thread motor_thread_data;

static void motor_thread_entry(void *, void *, void *);

void motor_control_start(void);

#endif