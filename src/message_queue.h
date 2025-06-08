#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#include <zephyr/kernel.h>

typedef enum {
    MSG_MOTOR_MOVE,
    MSG_MOTOR_COMPLETE,
    MSG_CAMERA_CAPTURE,
    MSG_CAMERA_COMPLETE,
    MSG_USB_COMMAND
} msg_type_t;

typedef struct {
    msg_type_t type;
    union {
        struct {
            int steps;
            int direction;
        } motor_cmd;
        struct {
            char command[64];
        } usb_cmd;
        // Add other message payloads
    } data;
} app_message_t;

// Queue handles
extern struct k_msgq usb_to_motor_queue;
extern struct k_msgq motor_to_camera_queue;
extern struct k_msgq camera_to_usb_queue;

void message_queue_init(void);

#endif