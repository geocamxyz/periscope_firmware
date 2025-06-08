#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include "usb_comm.h"
#include "message_queue.h"

#define USB_THREAD_STACK_SIZE 1024
#define USB_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(usb_thread_stack, USB_THREAD_STACK_SIZE);
static struct k_thread usb_thread_data;

static void usb_thread_entry(void *, void *, void *);

void usb_comm_start(void)
{
    // Initialize USB CDC
    usb_enable(NULL);

    k_thread_create(&usb_thread_data, usb_thread_stack,
                    K_THREAD_STACK_SIZEOF(usb_thread_stack),
                    usb_thread_entry, NULL, NULL, NULL,
                    USB_THREAD_PRIORITY, 0, K_NO_WAIT);
}

static void usb_thread_entry(void *p1, void *p2, void *p3)
{
    app_message_t msg;
    char rx_buffer[64];

    while (1) {
        // Read USB CDC data
        if (read_usb_command(rx_buffer, sizeof(rx_buffer)) > 0) {
            // Parse command and create message
            msg.type = MSG_USB_COMMAND;
            strcpy(msg.data.usb_cmd.command, rx_buffer);

            // Send to motor control
            k_msgq_put(&usb_to_motor_queue, &msg, K_NO_WAIT);
        }

        // Check for responses from other subsystems
        if (k_msgq_get(&camera_to_usb_queue, &msg, K_NO_WAIT) == 0) {
            send_usb_response("Camera operation complete");
        }

        k_sleep(K_MSEC(10));
    }
}