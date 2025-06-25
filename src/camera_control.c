#include <zephyr/drivers/spi.h>
#include "camera_control.h"
#include "gnss.h"
#include "messaging.h"
#include <zephyr/logging/log.h>

#define CAMERA_THREAD_STACK_SIZE 1024
#define CAMERA_THREAD_PRIORITY 7

LOG_MODULE_DECLARE(gc_periscope, LOG_LEVEL_DBG);

K_THREAD_STACK_DEFINE(camera_thread_stack, CAMERA_THREAD_STACK_SIZE);
static struct k_thread camera_thread_data;
static const struct gpio_dt_spec trigger_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(cam_trigger), gpios);
static const struct gpio_dt_spec power_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(cam_power), gpios);

K_TIMER_DEFINE(exposure_timer, NULL, NULL);

static void trigger_callback(struct k_timer *timer_id)
{
    gpio_pin_set_dt(&trigger_pin, 0);
}

K_TIMER_DEFINE(trigger_timer, trigger_callback, NULL);

void camera_init_hardware(void) {
    gpio_pin_configure_dt(&trigger_pin, GPIO_OUTPUT);
}

void camera_control_start(void) {
    // Initialize camera hardware
    camera_init_hardware();

    k_thread_create(&camera_thread_data, camera_thread_stack,
                    K_THREAD_STACK_SIZEOF(camera_thread_stack),
                    camera_thread_entry, NULL, NULL, NULL,
                    CAMERA_THREAD_PRIORITY, 0, K_NO_WAIT);
}

int32_t exposure_time_us = 500000;

void camera_trigger(void) {
    const usb_msg_t msg = {
        .type = MSG_CAMERA_TRIGGER,
        .pps_offset_us = get_pps_offset_us(),
        .data = {0}
    };
    k_msgq_put(&usb_queue, &msg, K_NO_WAIT);

    gpio_pin_set_dt(&trigger_pin, 1);
    // turn pin off after 250ns
    k_timer_start(&trigger_timer, K_NSEC(250), K_NO_WAIT);
    k_timer_start(&exposure_timer, K_USEC(exposure_time_us), K_NO_WAIT);
}

void camera_thread_entry(void *p1, void *p2, void *p3) {
    while (1) {
        k_event_wait(&system_events, EVENT_START_CAPTURE, true, K_FOREVER);
        atomic_inc(&threads_running);

        while (atomic_get(&system_capturing)) {
            uint32_t events = k_event_wait(&system_events, EVENT_POSITIONED | EVENT_STOP_CAPTURE, false, K_FOREVER);
            if (events & EVENT_STOP_CAPTURE) {
                break;
            }

            k_event_clear(&system_events, EVENT_POSITIONED);

            camera_trigger();

            // wait for exposure to finish
            k_timer_status_sync(&exposure_timer);
            k_event_post(&system_events, EVENT_EXPOSURE_COMPLETE);
        }

        k_timer_stop(&exposure_timer);
        atomic_dec(&threads_running);
        k_event_post(&system_events, EVENT_THREAD_STOPPED);
    }
}
