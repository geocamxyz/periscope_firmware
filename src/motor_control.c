#include <zephyr/drivers/gpio.h>
#include "motor_control.h"
#include <stdlib.h>
#include "messaging.h"
#include "tmc_uart.h"

// Define the global variables here
const struct gpio_dt_spec motor_index = GPIO_DT_SPEC_GET(DT_NODELABEL(motor_index), gpios);
volatile int32_t motor_step_counter = 0;
struct gpio_callback step_cb_data;
const struct device *const motor_uart = DEVICE_DT_GET(MOTOR_UART_NODE);
K_THREAD_STACK_DEFINE(motor_thread_stack, MOTOR_THREAD_STACK_SIZE);
struct k_thread motor_thread_data;

void step_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    motor_step_counter++;
}

void set_velocity(int32_t velocity) {
    uart_write_register(motor_uart, 0x00, 0x22, velocity);
}

void go_to_position(int32_t target_steps) {
    int32_t steps_to_go = target_steps - motor_step_counter;

    if (steps_to_go == 0) {
        set_velocity(0);
        return;
    }

    // Direction: positive velocity for forward, negative for reverse
    int32_t direction = (steps_to_go > 0) ? 1 : -1;
    int32_t abs_steps_to_go = abs(steps_to_go);

    // Ramp parameters
    const int32_t max_velocity = 1000;     // Adjust for your motor
    const int32_t accel_steps = 200;       // Steps to reach max velocity
    const int32_t min_velocity = 50;       // Minimum velocity to maintain motion

    while (abs(motor_step_counter - target_steps) > 1) {
        int32_t remaining_steps = abs(target_steps - motor_step_counter);

        int32_t velocity;

        // Acceleration phase
        if (abs(motor_step_counter - (target_steps - steps_to_go)) < accel_steps) {
            velocity = min_velocity +
                      ((max_velocity - min_velocity) *
                       abs(motor_step_counter - (target_steps - steps_to_go))) / accel_steps;
        }
        // Deceleration phase
        else if (remaining_steps < accel_steps) {
            velocity = min_velocity +
                      ((max_velocity - min_velocity) * remaining_steps) / accel_steps;
        }
        // Constant velocity phase
        else {
            velocity = max_velocity;
        }

        set_velocity(velocity * direction);
        k_msleep(10);  // Small delay for control loop
    }

    set_velocity(0);  // Stop at target
}

void motor_init_hardware(void) {
    gpio_pin_configure_dt(&motor_index, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&motor_index, GPIO_INT_EDGE_RISING);

    gpio_init_callback(&step_cb_data, step_interrupt_handler, BIT(motor_index.pin));
    gpio_add_callback(motor_index.port, &step_cb_data);
}

void motor_control_start(void) {
    motor_init_hardware();

    k_thread_create(&motor_thread_data, motor_thread_stack,
                    K_THREAD_STACK_SIZEOF(motor_thread_stack),
                    motor_thread_entry, NULL, NULL, NULL,
                    MOTOR_THREAD_PRIORITY, 0, K_NO_WAIT);
}

void motor_thread_entry(void *p1, void *p2, void *p3) {
    while (1) {
        k_sleep(K_MSEC(100)); // Placeholder
    }
}