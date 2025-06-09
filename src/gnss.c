#include <zephyr/device.h>
#include "gnss.h"
#include "messaging.h"

const struct gpio_dt_spec gnss_pps_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(gnss_pps), gpios);
volatile uint64_t last_pps_timestamp = 0;
struct gpio_callback pps_cb_data;
const struct device *const gnss_uart = DEVICE_DT_GET(GNSS_UART_NODE);
K_THREAD_STACK_DEFINE(gnss_thread_stack, GNSS_THREAD_STACK_SIZE);
struct k_thread gnss_thread_data;


static void pps_isr_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    last_pps_timestamp = k_cycle_get_64();
}


void gnss_init_hardware(void) {
    if (!device_is_ready(gnss_uart)) {
        printk("GNSS device not ready\n");
    }

    gpio_pin_configure_dt(&gnss_pps_pin, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&gnss_pps_pin, GPIO_INT_EDGE_RISING);

    gpio_init_callback(&pps_cb_data, pps_isr_handler, BIT(gnss_pps_pin.pin));
    gpio_add_callback(gnss_pps_pin.port, &pps_cb_data);
}


// Convert cycles to microseconds since last PPS
uint32_t get_pps_offset_us(void) {
    uint64_t now = k_cycle_get_64();
    uint64_t cycles_since_pps = now - last_pps_timestamp;

    // Convert to microseconds
    return (uint32_t) (k_cyc_to_us_floor64(cycles_since_pps));
}


void gnss_start(void) {
    gnss_init_hardware();

    k_thread_create(&gnss_thread_data, gnss_thread_stack,
                    K_THREAD_STACK_SIZEOF(gnss_thread_stack),
                    gnss_thread_entry, NULL, NULL, NULL,
                    GNSS_THREAD_PRIORITY, 0, K_NO_WAIT);
}

void gnss_thread_entry(void *p1, void *p2, void *p3) {
    while (1) {
        k_sleep(K_MSEC(1000));
    }
}
