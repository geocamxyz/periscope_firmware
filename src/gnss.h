#ifndef GNSS_H
#define GNSS_H

#include <zephyr/kernel.h>
#include "zephyr/drivers/gpio.h"

#define GNSS_UART_NODE DT_ALIAS(gnss_uart)
#define GNSS_THREAD_STACK_SIZE 1024
#define GNSS_THREAD_PRIORITY 7

// Declare as extern - definitions will be in gnss.c
extern const struct gpio_dt_spec gnss_pps_pin;
extern volatile uint64_t last_pps_timestamp;
extern struct gpio_callback pps_cb_data;
extern const struct device *const gnss_uart;
extern struct k_thread gnss_thread_data;

// Function declarations
void gnss_thread_entry(void *, void *, void *);
void gnss_start(void);
uint32_t get_pps_offset_us(void);

#endif