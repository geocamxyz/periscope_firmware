#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include "usb_comm.h"
#include "messaging.h"

#define USB_THREAD_STACK_SIZE 1024
#define USB_THREAD_PRIORITY 5

LOG_MODULE_DECLARE(gc_periscope, LOG_LEVEL_DBG);

K_THREAD_STACK_DEFINE(usb_thread_stack, USB_THREAD_STACK_SIZE);
static struct k_thread usb_thread_data;

#ifdef CONFIG_BOARD_NATIVE_SIM
const struct device *data_dev = DEVICE_DT_GET(DT_NODELABEL(uart_console));
#else
const struct device *data_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart1));
#endif

static void usb_thread_entry(void *, void *, void *);

void usb_comm_start(void)
{

    LOG_INF("USB initialized");

    k_thread_create(&usb_thread_data, usb_thread_stack,
                    K_THREAD_STACK_SIZEOF(usb_thread_stack),
                    usb_thread_entry, NULL, NULL, NULL,
                    USB_THREAD_PRIORITY, 0, K_NO_WAIT);
}

// Message queue for incoming UART data events
K_MSGQ_DEFINE(uart_rx_queue, sizeof(uint8_t), 32, 1);

// UART interrupt callback
static void uart_isr_callback(const struct device *dev, void *user_data)
{
    uint8_t byte;

    if (!uart_irq_update(dev)) {
        return;
    }

    // Handle received data
    if (uart_irq_rx_ready(dev)) {
        while (uart_fifo_read(dev, &byte, 1) == 1) {
            // Put received byte into message queue (non-blocking)
            if (k_msgq_put(&uart_rx_queue, &byte, K_NO_WAIT) != 0) {
                LOG_WRN("UART RX queue full, dropping byte: 0x%02x", byte);
            }
        }
    }
}

static void handle_uart_data(uint8_t data)
{
    LOG_INF("Received UART byte: 0x%02x ('%c')", data,
            (data >= 32 && data <= 126) ? data : '.');

    // Generate events based on received data
    switch (data) {
        case 's':
            k_event_post(&system_events, EVENT_REQ_START);
            LOG_INF("Event: Start command received");
            // Handle start event
            break;
        case 'S':
            k_event_post(&system_events, EVENT_REQ_STOP);
            LOG_INF("Event: Stop command received");
            // Handle stop event
            break;
        default:
            LOG_DBG("Event: Unknown command: 0x%02x", data);
            break;
    }
}



static void usb_thread_entry(void *p1, void *p2, void *p3)
{
    usb_msg_t msg;
    uint8_t rx_byte;

    uart_irq_callback_user_data_set(data_dev, uart_isr_callback, NULL);
    uart_irq_rx_enable(data_dev);

    while (1) {
        // Check for responses from other subsystems
        if (k_msgq_get(&usb_queue, &msg, K_NO_WAIT) == 0) {
            uart_tx(data_dev, (uint8_t*)&msg, sizeof(msg), SYS_FOREVER_MS);
        }

        if (k_msgq_get(&uart_rx_queue, &rx_byte, K_NO_WAIT) == 0) {
            handle_uart_data(rx_byte);
        }

        k_sleep(K_MSEC(10));
    }
}