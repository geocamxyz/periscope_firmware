#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include "motor_control.h"

#include <math.h>
#include <stdlib.h>
#include "messaging.h"
#include "tmc2209.h"

#define MICROSTEPS 8
#define STEPS_PER_MOVE 200  // 200 full steps per discrete move
#define MOVE_TIME_MS 250    // Time for each discrete move

LOG_MODULE_REGISTER(motor_control, LOG_LEVEL_DBG);

// S-curve motion profile
struct scurve_profile {
    uint32_t total_steps;      // Total steps to move
    uint32_t accel_steps;      // Steps during acceleration
    uint32_t decel_steps;      // Steps during deceleration
    uint32_t const_steps;      // Steps at constant velocity

    double max_velocity;       // Maximum velocity (steps/sec)
    double acceleration;       // Acceleration (steps/sec²)
    double jerk;              // Jerk (steps/sec³)

    uint32_t step_count;      // Current step count
    uint64_t start_time;      // Movement start time
};

// Define the global variables
const struct gpio_dt_spec motor_diag = GPIO_DT_SPEC_GET(DT_NODELABEL(motor_diagnostic), gpios);
const struct gpio_dt_spec motor_step = GPIO_DT_SPEC_GET(DT_NODELABEL(motor_step), gpios);
const struct gpio_dt_spec motor_dir = GPIO_DT_SPEC_GET(DT_NODELABEL(motor_dir), gpios);
const struct gpio_dt_spec motor_enable = GPIO_DT_SPEC_GET(DT_NODELABEL(motor_enable), gpios);
const struct gpio_dt_spec home_enable = GPIO_DT_SPEC_GET(DT_NODELABEL(home_on), gpios);
const struct gpio_dt_spec home_detect = GPIO_DT_SPEC_GET(DT_NODELABEL(home_det), gpios);

volatile int32_t motor_position = 0;  // Current position in microsteps
volatile bool home = false;
volatile bool movement_active = false;

// Callbacks and thread management
static struct gpio_callback home_cb_data;
static struct gpio_callback diag_cb_data;
static struct k_timer step_timer;
static struct scurve_profile profile;
static int32_t move_direction = 1;

const struct device *const motor_uart = DEVICE_DT_GET(MOTOR_UART_NODE);
K_THREAD_STACK_DEFINE(motor_thread_stack, MOTOR_THREAD_STACK_SIZE);
struct k_thread motor_thread_data;

uint32_t generate_chopconf(
    uint8_t diss2vs,     // Bit 31: Low side short protection disable (0-1)
    uint8_t diss2g,      // Bit 30: Short to GND protection disable (0-1)
    uint8_t dedge,       // Bit 29: Enable double edge step pulses (0-1)
    uint8_t intpol,      // Bit 28: Interpolation to 256 microsteps (0-1)
    uint8_t mres,        // Bits 27-24: Micro step resolution (0-15)
    uint8_t vsense,      // Bit 17: Sense resistor voltage based current scaling (0-1)
    uint8_t tbl,         // Bits 16-15: Blank time select (0-3)
    uint8_t hend,        // Bits 11-7: Hysteresis low value (0-15)
    uint8_t hstrt,       // Bits 6-4: Hysteresis start value (0-7)
    uint8_t toff         // Bits 3-0: Off time (0-15)
) {
    // Validate input ranges
    if (diss2vs > 1) diss2vs = 1;
    if (diss2g > 1) diss2g = 1;
    if (dedge > 1) dedge = 1;
    if (intpol > 1) intpol = 1;
    if (mres > 15) mres = 15;
    if (vsense > 1) vsense = 1;
    if (tbl > 3) tbl = 3;
    if (hend > 15) hend = 15;
    if (hstrt > 7) hstrt = 7;
    if (toff > 15) toff = 15;

    uint32_t result = 0;

    result |= ((uint32_t)(diss2vs & 0x01) << 31);
    result |= ((uint32_t)(diss2g & 0x01) << 30);
    result |= ((uint32_t)(dedge & 0x01) << 29);
    result |= ((uint32_t)(intpol & 0x01) << 28);
    result |= ((uint32_t)(mres & 0x0F) << 24);
    result |= ((uint32_t)(vsense & 0x01) << 17);
    result |= ((uint32_t)(tbl & 0x03) << 15);
    result |= ((uint32_t)(hend & 0x0F) << 7);
    result |= ((uint32_t)(hstrt & 0x07) << 4);
    result |= (toff & 0x0F);

    return result;
}

uint32_t ihold_irun(uint8_t ihold, uint8_t irun, uint8_t iholddelay) {
    if (ihold > 31) ihold = 31;
    if (irun > 31) irun = 31;
    if (iholddelay > 15) iholddelay = 15;

    uint16_t result = 0;
    result |= (ihold & 0x1F);
    result |= ((irun & 0x1F) << 8);
    result |= ((iholddelay & 0x0F) << 13);

    return result;
}

void home_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    home = true;
    LOG_INF("Home detected");
}

void diag_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    LOG_INF("Motor diagnostic triggered");
}

void check_motor_status() {
    uint32_t value;

    if (tmc2209_read_register(motor_uart, 0, 0x6F, &value) == 0) {
        LOG_INF("DRV_STATUS: 0x%08X", value);
        if (value & (1 << 31)) LOG_WRN("Standstill detected");
        if (value & (1 << 29)) LOG_WRN("Motor load detected");
    }

    if (tmc2209_read_register(motor_uart, 0, 0x06, &value) == 0) {
        LOG_INF("IOIN: 0x%08X", value);
    }

    if (tmc2209_read_register(motor_uart, 0, 0x01, &value) == 0) {
        LOG_INF("GSTAT: 0x%08X", value);
    }
}

void configure_motor() {
    /*
     * Configure TMC2209 for step/dir interface:
     * - External step/dir control
     * - Internal voltage reference
     * - Microstepping enabled
     */
    tmc2209_write_register(motor_uart, 0x00, 0x00, 0b0000100000); // Enable step/dir mode

    // Set current levels
    // tmc2209_write_register(motor_uart, 0x00, 0x10, ihold_irun(31, 31, 8));

    // Configure chopper
    tmc2209_write_register(motor_uart, 0x00, 0x6C, generate_chopconf(
        0,    // diss2vs: Short protection enabled
        0,    // diss2g: Short to GND protection enabled
        0,    // dedge: Normal step pulses
        1,    // intpol: Enable interpolation
        MRES_016, // microstep resolution
        0,    // vsense: Low sensitivity
        1,    // tbl: 24 clocks blank time
        12,   // hend: Hysteresis offset
        8,    // hstrt: Hysteresis start
        3     // toff: Off time
    ));

    // Power management
    tmc2209_write_register(motor_uart, 0x00, 0x11, 0); // No power down

    // Stallguard threshold
    tmc2209_write_register(motor_uart, 0x00, 0x40, 40);

    k_msleep(1);
}

void motor_init_hardware(void) {
    // Configure GPIO pins
    gpio_pin_configure_dt(&motor_dir, GPIO_OUTPUT);
    gpio_pin_configure_dt(&motor_enable, GPIO_OUTPUT);

    gpio_pin_configure_dt(&home_detect, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&home_detect, GPIO_INT_EDGE_RISING);
    gpio_init_callback(&home_cb_data, home_interrupt_handler, BIT(home_detect.pin));
    gpio_add_callback(home_detect.port, &home_cb_data);

    gpio_pin_configure_dt(&motor_diag, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&motor_diag, GPIO_INT_EDGE_RISING);
    gpio_init_callback(&diag_cb_data, diag_interrupt_handler, BIT(motor_diag.pin));
    gpio_add_callback(motor_diag.port, &diag_cb_data);

    // Set initial pin states
    gpio_pin_set_dt(&motor_dir, 0);
}

void motor_control_start(void) {
    tmc2209_init_uart(motor_uart);
    motor_init_hardware();

    gpio_pin_set_dt(&motor_enable, 0); // Initially disabled

    k_thread_create(&motor_thread_data, motor_thread_stack,
                    K_THREAD_STACK_SIZEOF(motor_thread_stack),
                    motor_thread_entry, NULL, NULL, NULL,
                    MOTOR_THREAD_PRIORITY, 0, K_NO_WAIT);
}

void motor_thread_entry(void *p1, void *p2, void *p3) {
    int32_t current_step = 0;
    int32_t direction = 1;
    const int32_t max_steps = 3; // Move 3 discrete steps in each direction

    while (1) {
        k_event_wait(&system_events, EVENT_START_CAPTURE, true, K_FOREVER);
        atomic_inc(&threads_running);
        gpio_pin_set_dt(&motor_enable, 1); // Enable motor
        configure_motor();

        while (atomic_get(&system_capturing)) {
            k_event_clear(&system_events, EVENT_EXPOSURE_COMPLETE);

            // Determine next position
            if (current_step >= max_steps) {
                direction = -1;
            } else if (current_step <= 0) {
                direction = 1;
            }

            LOG_INF("Moving motor discrete step %d (direction %d)", current_step, direction);

#ifndef CONFIG_BOARD_NATIVE_SIM
            uint64_t move_start = k_cycle_get_64();

            // Move one discrete step
            // move_discrete_step(direction);
            k_event_post(&system_events, EVENT_TRIGGER_STEPS);
            k_event_wait(&system_events, EVENT_STEPS_COMPLETE, true, K_FOREVER);
            // wait_for_movement_complete();

            uint64_t move_end = k_cycle_get_64();
            LOG_INF("Discrete move took %d us", (int32_t)k_cyc_to_us_floor64(move_end - move_start));
#endif
            current_step += direction;

            if (!atomic_get(&system_capturing)) {
                break;
            }

            k_event_post(&system_events, EVENT_POSITIONED);
            k_event_wait(&system_events, EVENT_EXPOSURE_COMPLETE | EVENT_STOP_CAPTURE, false, K_FOREVER);
        }

        gpio_pin_set_dt(&motor_enable, 0); // Disable motor
        atomic_dec(&threads_running);
        k_event_post(&system_events, EVENT_THREAD_STOPPED);
    }
}