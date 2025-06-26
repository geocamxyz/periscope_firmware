//
// Created by linus on 6/26/25.
//

#include "stepper.h"
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include "scurve_delays.h"

#include "messaging.h"
#include "zephyr/logging/log.h"

#define STEP_PIN 2

LOG_MODULE_REGISTER(pio_stepper, LOG_LEVEL_DBG);

RPI_PICO_PIO_DEFINE_PROGRAM(step_pulse, 0, 9,
            //     .wrap_target
    0x80a0, //  0: pull   block
    0xe041, //  1: set    y, 1
    0x6030, //  2: out    x, 16
    0xee01, //  3: set    pins, 1                [14]
    0xec00, //  4: set    pins, 0                [12]
    0x0029, //  5: jmp    !x, 9
    0x0047, //  6: jmp    x--, 7
    0xbd42, //  7: nop                           [29]
    0x0047, //  8: jmp    x--, 7
    0x0082, //  9: jmp    y--, 2
            //     .wrap
);

K_THREAD_STACK_DEFINE(stepper_thread_stack, STEPPER_THREAD_STACK_SIZE);
struct k_thread stepper_thread_data;

// Synchronization primitives
static struct k_sem sequence_trigger_sem;
static struct k_mutex stepper_mutex;
static volatile bool sequence_running = false;

static int pio_init(PIO pio, uint32_t sm, uint32_t pins, uint offset, float div)
{
    pio_sm_config sm_config;

    sm_config = pio_get_default_sm_config();
    sm_config_set_set_pins(&sm_config, pins, 1);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&sm_config, div);
    sm_config_set_wrap(&sm_config,
               offset + RPI_PICO_PIO_GET_WRAP_TARGET(step_pulse),
               offset + RPI_PICO_PIO_GET_WRAP(step_pulse));

    pio_gpio_init(pio, pins);
    pio_sm_set_consecutive_pindirs(pio, sm, pins, 1, true);
    pio_sm_init(pio, sm, offset, &sm_config);
    pio_sm_set_enabled(pio, sm, true);

    return 0;
}

void stepper_control_start(void)
{
    // Initialize synchronization primitives
    k_sem_init(&sequence_trigger_sem, 0, 1);
    k_mutex_init(&stepper_mutex);

    k_thread_create(&stepper_thread_data, stepper_thread_stack,
                    K_THREAD_STACK_SIZEOF(stepper_thread_stack),
                    stepper_thread_entry, NULL, NULL, NULL,
                    STEPPER_THREAD_PRIORITY, 0, K_NO_WAIT);
}

// Function to trigger pulse sequence from another thread
int stepper_trigger_sequence(void)
{
    k_mutex_lock(&stepper_mutex, K_FOREVER);

    if (sequence_running) {
        k_mutex_unlock(&stepper_mutex);
        return -EBUSY;  // Sequence already running
    }

    k_mutex_unlock(&stepper_mutex);

    // Signal the stepper thread to start the sequence
    k_sem_give(&sequence_trigger_sem);

    return 0;
}

// Function to check if sequence is currently running
bool stepper_is_sequence_running(void)
{
    k_mutex_lock(&stepper_mutex, K_FOREVER);
    bool running = sequence_running;
    k_mutex_unlock(&stepper_mutex);
    return running;
}

// Function to execute the pulse sequence
static void execute_pulse_sequence(PIO pio, uint sm)
{
    k_mutex_lock(&stepper_mutex, K_FOREVER);
    sequence_running = true;
    k_mutex_unlock(&stepper_mutex);

    // Process pairs of delays (1600 delays = 800 FIFO words)
    for (int i = 0; i < PULSE_SEQUENCE_LENGTH; i += 2) {
        // Pack two 16-bit delays into one 32-bit word
        // Upper 16 bits: first delay, Lower 16 bits: second delay
        uint32_t pulse_data = ((uint32_t)pulse_delays[i] << 16) | pulse_delays[i + 1];

        // Check FIFO level - if it's getting full, we can yield to other threads
        uint fifo_level = pio_sm_get_tx_fifo_level(pio, sm);

        if (fifo_level >= 6) {  // FIFO is nearly full (6/8 entries)
            // Calculate approximate time until FIFO has space
            // Each pulse takes: (delay_cycles * clock_period) + fixed_overhead
            // With clock div of 4.0, each cycle is 4 * (1/125MHz) = 32ns
            // Rough estimate: use the current delays to predict timing
            uint32_t delay1 = pulse_delays[i];
            uint32_t delay2 = pulse_delays[i + 1];

            // Estimate time for these two pulses (in microseconds)
            // delay_cycles * 32ns + overhead (~1us per pulse)
            uint32_t estimated_time_us = ((delay1 + delay2) * 32) / 1000 + 2;

            // Yield for a portion of this time to let other threads run
            if (estimated_time_us > 50) {  // Only yield if there's meaningful time
                k_usleep(estimated_time_us * 2 / 3);  // Yield for half the estimated time
            } else {
                k_yield();  // Quick yield for shorter pulses
            }
        }

        // Send the packed pulse data to PIO (may block if FIFO is full)
        pio_sm_put_blocking(pio, sm, pulse_data);
    }

    k_mutex_lock(&stepper_mutex, K_FOREVER);
    sequence_running = false;
    k_mutex_unlock(&stepper_mutex);
}

void stepper_thread_entry(void*, void*, void*)
{
    PIO pio;
    uint sm;
    uint offset;

    pio_claim_free_sm_and_add_program_for_gpio_range(&step_pulse_program, &pio, &sm, &offset, STEP_PIN, 1, true);
    pio_init(pio, sm, STEP_PIN, offset, 4.0f);

    while (true) {
        // Wait for trigger signal from another thread
        k_event_wait(&system_events, EVENT_TRIGGER_STEPS, true, K_FOREVER);
        // k_sem_take(&sequence_trigger_sem, K_FOREVER);

        // Execute the pulse sequence
        execute_pulse_sequence(pio, sm);
        LOG_INF("Steps complete - sending completion");
        k_event_post(&system_events, EVENT_STEPS_COMPLETE);
    }
}