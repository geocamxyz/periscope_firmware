#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include "usb_comm.h"
#include "messaging.h"

#define USB_THREAD_STACK_SIZE 1024
#define USB_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(usb_thread_stack, USB_THREAD_STACK_SIZE);
static struct k_thread usb_thread_data;

const struct device *data_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart1));

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
    usb_msg_t msg;

    while (1) {
        // Check for responses from other subsystems
        if (k_msgq_get(&usb_queue, &msg, K_NO_WAIT) == 0) {
            uart_tx(data_dev, (uint8_t*)&msg, sizeof(msg), SYS_FOREVER_MS);
        }

        k_sleep(K_MSEC(10));
    }
}