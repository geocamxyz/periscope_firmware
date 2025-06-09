# periscope firmware

This is responsible for:
- motor pointing + feedback
- homing
- camera trigger
- precise time-marking (using gnss + PPS) of:
  - trigger events
  - imu data
  - temperature data

## Process
```mermaid
graph TD
    start --> home
    home --> start_capture[start capture]
    start_capture --> goto[go to position]
    goto --> triggercam[trigger camera]
    triggercam --> wait[wait for exposure]
    wait --> goto
    start_capture --> poll_sensors[poll sensors]
    poll_sensors --> send_data[send stamped data via usb]
    send_data --> poll_sensors
```

I am using Zephyr events to tell when to go between these states - and for each thread to know when to do what.