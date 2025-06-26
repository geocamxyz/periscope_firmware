#ifndef MESSAGING_H
#define MESSAGING_H

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>


#define EVENT_START_CAPTURE     BIT(0)
#define EVENT_STOP_CAPTURE      BIT(1)
#define EVENT_THREAD_STOPPED    BIT(2)
#define EVENT_REQ_START         BIT(3)
#define EVENT_REQ_STOP          BIT(4)

// When the motor has finished moving
#define EVENT_POSITIONED        BIT(5)

// When the camera has finished the exposure
#define EVENT_EXPOSURE_COMPLETE BIT(6)

#define EVENT_TRIGGER_STEPS     BIT(7)
#define EVENT_STEPS_COMPLETE    BIT(8)

extern struct k_event system_events;
extern atomic_t system_capturing;
extern atomic_t threads_running;

// Message queue definitions
#define MSG_QUEUE_SIZE 10
#define MSG_QUEUE_ALIGN 4

// USB message structure
typedef struct {
    uint8_t type;           // Message type identifier
    uint32_t pps_offset_us; // Microseconds since last PPS pulse
    uint8_t data[8];        // Additional data
} __packed usb_msg_t;

// Message types
#define MSG_PPS_MARK          0x00 // used to say what the time is
#define MSG_CAMERA_TRIGGER    0x01
#define MSG_IMU_ACC_DATA      0x02
#define MSG_IMU_GYRO_DATA     0x03
#define MSG_IMU_MAG_DATA      0x04

extern struct k_msgq usb_queue;

void message_queue_init(void);

void start_capture(void);
void stop_capture(void);

#endif