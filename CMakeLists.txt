cmake_minimum_required(VERSION 3.20.0)
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(periscope_firmware)

target_sources(app PRIVATE
        src/main.c
        src/messaging.c
        src/usb_comm.c
        src/gnss.c
        src/motor_control.c
        src/camera_control.c
)
