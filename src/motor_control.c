#include <zephyr/drivers/gpio.h>
#include "motor_control.h"
#include "message_queue.h"

void step_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    motor_step_counter++;
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

static void motor_thread_entry(void *p1, void *p2, void *p3) {
    app_message_t msg;

    while (1) {
        // Wait for motor commands
        if (k_msgq_get(&usb_to_motor_queue, &msg, K_FOREVER) == 0) {
            if (msg.type == MSG_USB_COMMAND) {
                // Parse motor command and execute
                execute_motor_movement(&msg.data.usb_cmd);

                // Notify camera when done
                msg.type = MSG_MOTOR_COMPLETE;
                k_msgq_put(&motor_to_camera_queue, &msg, K_NO_WAIT);
            }
        }
    }
}
