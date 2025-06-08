#include <zephyr/drivers/spi.h>
#include "camera_control.h"
#include "message_queue.h"

#define CAMERA_THREAD_STACK_SIZE 1024
#define CAMERA_THREAD_PRIORITY 7

K_THREAD_STACK_DEFINE(camera_thread_stack, CAMERA_THREAD_STACK_SIZE);
static struct k_thread camera_thread_data;

static void camera_thread_entry(void *, void *, void *);

void camera_control_start(void) {
    // Initialize camera hardware
    camera_init_hardware();

    k_thread_create(&camera_thread_data, camera_thread_stack,
                    K_THREAD_STACK_SIZEOF(camera_thread_stack),
                    camera_thread_entry, NULL, NULL, NULL,
                    CAMERA_THREAD_PRIORITY, 0, K_NO_WAIT);
}

static void camera_thread_entry(void *p1, void *p2, void *p3) {
    app_message_t msg;

    while (1) {
        // Wait for motor completion signal
        if (k_msgq_get(&motor_to_camera_queue, &msg, K_FOREVER) == 0) {
            if (msg.type == MSG_MOTOR_COMPLETE) {
                // Take photo
                capture_image();

                // Notify USB thread we're done
                msg.type = MSG_CAMERA_COMPLETE;
                k_msgq_put(&camera_to_usb_queue, &msg, K_NO_WAIT);
            }
        }
    }
}
