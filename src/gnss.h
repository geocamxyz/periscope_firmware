#ifndef GNSS_H
#define GNSS_H

#include <zephyr/kernel.h>

#define GNSS_UART_NODE DT_ALIAS(gnss_uart)

#define GNSS_THREAD_STACK_SIZE 1024
#define GNSS_THREAD_PRIORITY 7
K_THREAD_STACK_DEFINE(gnss_thread_stack, GNSS_THREAD_STACK_SIZE);

static const struct device *const gnss_uart = DEVICE_DT_GET(GNSS_UART_NODE);

static struct k_thread gnss_thread_data;

static void gnss_thread_entry(void *, void *, void *);

void gnss_start(void);

#endif