#include <zephyr/drivers/spi.h>
#include "camera_control.h"
#include "messaging.h"

#define CAMERA_THREAD_STACK_SIZE 1024
#define CAMERA_THREAD_PRIORITY 7

K_THREAD_STACK_DEFINE(camera_thread_stack, CAMERA_THREAD_STACK_SIZE);
static struct k_thread camera_thread_data;

K_TIMER_DEFINE(exposure_timer, NULL, NULL);

void camera_init_hardware(void) {

}

void camera_control_start(void) {
    // Initialize camera hardware
    camera_init_hardware();

    k_thread_create(&camera_thread_data, camera_thread_stack,
                    K_THREAD_STACK_SIZEOF(camera_thread_stack),
                    camera_thread_entry, NULL, NULL, NULL,
                    CAMERA_THREAD_PRIORITY, 0, K_NO_WAIT);
}

int32_t exposure_time_ms = 100;

void camera_trigger(void) {
    k_timer_start(&exposure_timer, K_MSEC(exposure_time_ms), K_NO_WAIT);
}

void camera_thread_entry(void *p1, void *p2, void *p3) {
    while (1) {
        k_event_wait(&system_events, EVENT_START_CAPTURE, false, K_FOREVER);
        atomic_inc(&threads_running);

        while (atomic_get(&system_capturing)) {
            k_event_wait(&system_events, EVENT_POSITIONED, false, K_FOREVER);

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
