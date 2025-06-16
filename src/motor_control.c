#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "motor_control.h"

#include <math.h>
#include <stdlib.h>
#include "messaging.h"
#include "tmc2209.h"

#define MICROSTEPS 8

LOG_MODULE_REGISTER(motor_control, LOG_LEVEL_DBG);

// Define the global variables here
const struct gpio_dt_spec motor_index = GPIO_DT_SPEC_GET(DT_NODELABEL(motor_index), gpios);
const struct gpio_dt_spec motor_diag = GPIO_DT_SPEC_GET(DT_NODELABEL(motor_diagnostic), gpios);
const struct gpio_dt_spec motor_step = GPIO_DT_SPEC_GET(DT_NODELABEL(motor_step), gpios);
const struct gpio_dt_spec motor_enable = GPIO_DT_SPEC_GET(DT_NODELABEL(motor_enable), gpios);
volatile int32_t motor_step_counter = 0;
volatile int32_t direction = 1; // positive
struct gpio_callback step_cb_data;
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
    if (mres > 15) mres = 15;        // 4-bit field
    if (vsense > 1) vsense = 1;
    if (tbl > 3) tbl = 3;            // 2-bit field
    if (hend > 15) hend = 15;        // 4-bit field
    if (hstrt > 7) hstrt = 7;        // 3-bit field
    if (toff > 15) toff = 15;        // 4-bit field

    uint32_t result = 0;

    // Set each bit field at its proper position
    result |= ((uint32_t)(diss2vs & 0x01) << 31);  // Bit 31
    result |= ((uint32_t)(diss2g & 0x01) << 30);   // Bit 30
    result |= ((uint32_t)(dedge & 0x01) << 29);    // Bit 29
    result |= ((uint32_t)(intpol & 0x01) << 28);   // Bit 28
    result |= ((uint32_t)(mres & 0x0F) << 24);     // Bits 27-24
    // Bits 23-18 are reserved (set to 0)
    result |= ((uint32_t)(vsense & 0x01) << 17);   // Bit 17
    result |= ((uint32_t)(tbl & 0x03) << 15);      // Bits 16-15
    // Bit 14-12 are reserved (set to 0)
    result |= ((uint32_t)(hend & 0x0F) << 7);      // Bits 11-7 (Note: 4 bits shifted to position 7)
    result |= ((uint32_t)(hstrt & 0x07) << 4);     // Bits 6-4
    result |= (toff & 0x0F);                       // Bits 3-0

    return result;
}


void step_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    motor_step_counter += direction;
}

void check_motor_status() {
    uint32_t value;

    // Check driver status
    if (tmc2209_read_register(motor_uart, 0, 0x6F, &value) == 0) {
        LOG_INF("DRV_STATUS: 0x%08X", value);
        if (value & (1 << 31)) LOG_WRN("Standstill detected");
        if (value & (1 << 29)) LOG_WRN("Motor load detected");
    }

    // Check pin status
    if (tmc2209_read_register(motor_uart, 0, 0x06, &value) == 0) {
        LOG_INF("IOIN: 0x%08X", value);
    }

    // Check global status
    if (tmc2209_read_register(motor_uart, 0, 0x01, &value) == 0) {
        LOG_INF("GSTAT: 0x%08X", value);
    }
}


uint32_t ihold_irun(uint8_t ihold, uint8_t irun, uint8_t iholddelay) {
    // Validate input ranges
    if (ihold > 31) ihold = 31;           // 5-bit max (0-31)
    if (irun > 31) irun = 31;             // 5-bit max (0-31)
    if (iholddelay > 15) iholddelay = 15; // 4-bit max (0-15)

    // Combine the values according to bit positions:
    // Bits 4..0:   IHOLD
    // Bits 12..8:  IRUN
    // Bits 19..16: IHOLDDELAY (but we only need 14 bits total)
    uint16_t result = 0;
    result |= (ihold & 0x1F);              // Bits 4..0
    result |= ((irun & 0x1F) << 8);        // Bits 12..8
    result |= ((iholddelay & 0x0F) << 13); // Bits 16..13 (shifted to fit 14-bit)

    return result;
}

void set_velocity(int32_t velocity) {
    // LOG_INF("VEL: %d", velocity);
    tmc2209_write_register(motor_uart, 0x00, 0x22, velocity);
}

void configure_motor() {
    /*
     * generate internal voltage for VREF (max current)
     * external sense resisors
     * spreadcycle PWM (swappable with SPREAD pin)
     * not inverted motor direction
     * index shows first microstep position (ignored)
     * index output is step pulses from internal generator
     * pdn uart input function disabled
     * microstep resolution by MRES register
     * multistep filtering
     * normal operation
     */
    tmc2209_write_register(motor_uart, 0x00, 0x00, 0b0111100000);
    // set ihold and irun
    tmc2209_write_register(motor_uart, 0x00, 0x10, ihold_irun(16,16,8));
    tmc2209_write_register(motor_uart, 0x00, 0x6C, generate_chopconf(
        0,    // diss2vs: Short protection enabled
        0,    // diss2g: Short to GND protection enabled
        0,    // dedge: Normal step pulses
        1,    // intpol: Enable interpolation (default)
        MRES_016,    // mres: 128 microsteps (0b0111)
        0,    // vsense: Low sensitivity
        1,    // tbl: 24 clocks blank time
        0,    // hend: Hysteresis offset 0
        5,    // hstrt: Add 5 to hysteresis (default for StealthChop)
        3     // toff: Off time 3 (default for StealthChop)
    ));
    k_msleep(1);
}

void go_to_position(int32_t target_steps) {
    int32_t steps_to_go = target_steps - motor_step_counter;

    if (steps_to_go == 0) {
        set_velocity(0);
        return;
    }

    // Direction: positive velocity for forward, negative for reverse
    direction = (steps_to_go > 0) ? 1 : -1;

    // Ramp parameters
    const int32_t max_velocity = 20000;                 // Adjust for your motor
    const int32_t accel_steps = 50 * MICROSTEPS;       // Steps to reach max velocity
    const int32_t decel_steps = 60 * MICROSTEPS;       // Steps to go from max to min
    const int32_t start_velocity = 1000;                    // Minimum velocity to maintain motion
    const int32_t stop_velocity = 600;                    // Minimum velocity to maintain motion
    const int32_t control_period_ms = 1;               // Control loop period

    int32_t prev_remaining_steps = abs(target_steps - motor_step_counter);

    while (abs(motor_step_counter - target_steps) > 1) {
        int32_t remaining_steps = abs(target_steps - motor_step_counter);
        if (remaining_steps > prev_remaining_steps) {
            LOG_DBG("went past target by %d usteps", remaining_steps);
            break; // stop immediately
        }

        int32_t velocity;

        // Acceleration phase
        if (abs(motor_step_counter - (target_steps - steps_to_go)) < accel_steps) {
            velocity = start_velocity +
                      ((max_velocity - start_velocity) *
                       abs(motor_step_counter - (target_steps - steps_to_go))) / accel_steps;
        }
        // Deceleration phase - calculate velocity for one control loop ahead
        else if (remaining_steps < decel_steps) {
            // Estimate current velocity from position
            int32_t current_velocity = stop_velocity +
                      ((max_velocity - stop_velocity) * remaining_steps) / decel_steps;

            // Estimate steps that will be traveled in next control period
            // 1399 is 1000 (ms) * (1 / 0.715) which is the velocity scaler
            int32_t predicted_steps_traveled = (current_velocity * control_period_ms) / 1399;

            // Calculate remaining steps after next control period
            int32_t predicted_remaining = remaining_steps - predicted_steps_traveled;
            if (predicted_remaining < 0) predicted_remaining = 0;

            // Set velocity based on predicted future position
            velocity = stop_velocity +
                      ((max_velocity - stop_velocity) * predicted_remaining) / decel_steps;
        }
        // Constant velocity phase
        else {
            velocity = max_velocity;
        }

        // LOG_DBG("remaining steps, velocity: %d, %d", remaining_steps, velocity * MICROSTEPS);
        // if (velocity <= 256) {
        //     set_velocity(velocity * direction);
        // } else {
            set_velocity(velocity * direction * MICROSTEPS);
        // }
        k_msleep(control_period_ms);  // Small delay for control loop
        prev_remaining_steps = remaining_steps;
        // check_motor_status();
    }

    set_velocity(0);  // Stop at target
}
void motor_init_hardware(void) {
    gpio_pin_configure_dt(&motor_index, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&motor_index, GPIO_INT_EDGE_RISING);

    gpio_pin_configure_dt(&motor_step, GPIO_OUTPUT);
    gpio_pin_configure_dt(&motor_enable, GPIO_OUTPUT);

    gpio_init_callback(&step_cb_data, step_interrupt_handler, BIT(motor_index.pin));
    gpio_add_callback(motor_index.port, &step_cb_data);
}

void motor_control_start(void) {
    tmc2209_init_uart(motor_uart);
    motor_init_hardware();

    gpio_pin_set_dt(&motor_enable, 0);

    k_thread_create(&motor_thread_data, motor_thread_stack,
                    K_THREAD_STACK_SIZEOF(motor_thread_stack),
                    motor_thread_entry, NULL, NULL, NULL,
                    MOTOR_THREAD_PRIORITY, 0, K_NO_WAIT);
}

void motor_thread_entry(void *p1, void *p2, void *p3) {
    int32_t target_position = 0;
    int32_t delta = 200;
    int32_t dir = 1;

    while (1) {
        k_event_wait(&system_events, EVENT_START_CAPTURE, true, K_FOREVER);
        atomic_inc(&threads_running);
        gpio_pin_set_dt(&motor_enable, 1);
        configure_motor();
        set_velocity(0);
        k_usleep(500);

        while (atomic_get(&system_capturing)) {
            // gpio_pin_set_dt(&motor_step, 1);
            // k_usleep(100);
            // gpio_pin_set_dt(&motor_step, 0);
            // k_usleep(100);
            // continue;
            k_event_clear(&system_events, EVENT_EXPOSURE_COMPLETE);

            if (target_position >= delta * 5) {
                dir = -1;
            } else if (target_position <= 0) {
                dir = 1;
            }
            LOG_INF("Moving motor to %d", target_position);
            #ifndef CONFIG_BOARD_NATIVE_SIM
            uint64_t move_start = k_cycle_get_64();
            go_to_position(target_position * MICROSTEPS);
            uint64_t move_end = k_cycle_get_64();
            LOG_INF("Took %d us to move motor", (int32_t) k_cyc_to_us_floor64(move_end - move_start));
            #endif
            target_position += dir * delta;

            if (!atomic_get(&system_capturing)) {
                break;
            }

            k_event_post(&system_events, EVENT_POSITIONED);
            k_event_wait(&system_events, EVENT_EXPOSURE_COMPLETE | EVENT_STOP_CAPTURE, false, K_FOREVER);
        }

        gpio_pin_set_dt(&motor_enable, 0);
        atomic_dec(&threads_running);
        k_event_post(&system_events, EVENT_THREAD_STOPPED);
    }
}