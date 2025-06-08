#include <zephyr/kernel.h>
#include "usb_comm.h"
#include "motor_control.h"
#include "camera_control.h"
#include "gnss.h"
#include "message_queue.h"

int main(void)
{
    // Initialize message queues
    message_queue_init();

    // Start all subsystem threads
    usb_comm_start();
    gnss_start();
    motor_control_start();
    camera_control_start();

    printk("Periscope firmware started\n");

    // Main thread can sleep or handle system-level tasks
    while (1) {
        k_sleep(K_SECONDS(1));
        // Optional: system health checks, watchdog, etc.
    }

    return 0;
}