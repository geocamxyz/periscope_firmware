#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include "usb_comm.h"
#include "motor_control.h"
#include "camera_control.h"
#include "gnss.h"
#include "messaging.h"

LOG_MODULE_REGISTER(gc_periscope, LOG_LEVEL_DBG);

int main(void)
{
    const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint32_t dtr = 0;
    int timeout_count = 0;
    const int max_timeout = 5; // 0.5 second timeout

    if (usb_enable(NULL)) {
        printk("Failed to enable USB");
        return 0;
    }

    printk("Waiting for USB to stabilize...\n");
    k_sleep(K_MSEC(500));

    /* Poll DTR with timeout */
    while (!dtr && timeout_count < max_timeout) {
        printk("Polling DTR on console (%d/%d)\n", timeout_count, max_timeout);
        uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
        timeout_count++;
        k_sleep(K_MSEC(100));
    }

    if (dtr) {
        printk("DTR detected, host connected\n");
    } else {
        printk("DTR timeout, continuing anyway\n");
    }


    // Initialize message queues
    message_queue_init();
    //
    // // Start all subsystem threads
    usb_comm_start();
    gnss_start();
    motor_control_start();
    camera_control_start();

    LOG_INF("Periscope firmware started\n");
    k_msleep(10);

    while (1) {
        start_capture();
        k_event_wait(&system_events, EVENT_REQ_STOP, true, K_FOREVER);
        stop_capture();
        k_event_wait(&system_events, EVENT_REQ_START, true, K_FOREVER);
    }

    return 0;
}