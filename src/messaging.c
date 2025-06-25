#include "messaging.h"
#include "usb_comm.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(gc_periscope, LOG_LEVEL_DBG);

K_EVENT_DEFINE(system_events);
K_MSGQ_DEFINE(usb_queue, sizeof(usb_msg_t), MSG_QUEUE_SIZE, MSG_QUEUE_ALIGN);

atomic_t system_capturing = ATOMIC_INIT(0);
atomic_t threads_running = ATOMIC_INIT(0);

void message_queue_init(void)
{
    LOG_INF("Initializing message queues");

    // Purge any existing messages (shouldn't be any, but just in case)
    k_msgq_purge(&usb_queue);

    LOG_INF("Message queues initialized successfully");
}

void msg_queue_purge_all(void)
{
    LOG_INF("Purging all message queues");

    k_msgq_purge(&usb_queue);
}

void start_capture(void)
{
    if (atomic_get(&system_capturing)) {
        LOG_INF("Already capturing\n");
        return;
    }

    LOG_INF("Starting capture...");
    atomic_set(&system_capturing, 1);
    k_event_post(&system_events, EVENT_START_CAPTURE);
}

void stop_capture(void)
{
    if (!atomic_get(&system_capturing)) {
        LOG_INF("Not currently capturing");
        return;
    }

    LOG_INF("Stopping capture...");

    // Get current thread count
    int expected_stops = atomic_get(&threads_running);

    // Signal stop
    atomic_set(&system_capturing, 0);
    k_event_post(&system_events, EVENT_STOP_CAPTURE);

    // Wait for all threads to stop
    for (int i = 0; i < expected_stops; i++) {
        k_event_wait(&system_events, EVENT_THREAD_STOPPED, false, K_SECONDS(5));
    }

    // Clear events for next time
    k_event_clear(&system_events, EVENT_START_CAPTURE | EVENT_THREAD_STOPPED);

    LOG_INF("All threads stopped\n");
}