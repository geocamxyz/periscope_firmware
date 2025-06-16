#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define MOTOR_THREAD_STACK_SIZE 1024
#define MOTOR_THREAD_PRIORITY 6
#define MOTOR_UART_NODE DT_ALIAS(motor_uart)

const static uint8_t MRES_256 = 0b0000;
const static uint8_t MRES_128 = 0b0001;
const static uint8_t MRES_064 = 0b0010;
const static uint8_t MRES_032 = 0b0011;
const static uint8_t MRES_016 = 0b0100;
const static uint8_t MRES_008 = 0b0101;
const static uint8_t MRES_004 = 0b0110;
const static uint8_t MRES_002 = 0b0111;
const static uint8_t MRES_001 = 0b1000;

// Declare as extern - definitions will be in motor_control.c
extern const struct gpio_dt_spec motor_index;
extern struct gpio_callback step_cb_data;
extern const struct device *const motor_uart;
extern struct k_thread motor_thread_data;

// Function declarations
void motor_thread_entry(void *, void *, void *);
void motor_control_start(void);

#endif