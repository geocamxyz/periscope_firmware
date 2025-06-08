#include <zephyr/device.h>
#include "zephyr/kernel.h"
#include "gnss.h"

#include "message_queue.h"

void gnss_init_hardware(void) {
    if (!device_is_ready(gnss_uart)) {
        printk("GNSS device not ready\n");
    }
}

void gnss_start(void) {
    gnss_init_hardware();

    k_thread_create(&gnss_thread_data, gnss_thread_stack,
                    K_THREAD_STACK_SIZEOF(gnss_thread_stack),
                    gnss_thread_entry, NULL, NULL, NULL,
                    GNSS_THREAD_PRIORITY, 0, K_NO_WAIT);
}

static void gnss_thread_entry(void *p1, void *p2, void *p3) {
    app_message_t msg;

    while (1) {
        // Wait for motor completion signal
        if (k_msgq_get(&motor_to_camera_queue, &msg, K_FOREVER) == 0) {
            if (msg.type == MSG_MOTOR_COMPLETE) {
                // Take photo
                // capture_image();

                // Notify USB thread we're done
                msg.type = MSG_CAMERA_COMPLETE;
                k_msgq_put(&camera_to_usb_queue, &msg, K_NO_WAIT);
            }
        }
    }
}
