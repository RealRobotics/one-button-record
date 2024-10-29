# One Button Record

This repo contains code to start and stop recording of video on a ROS2 robot.  The robot must be fitted with a push button switch and a means of displaying three status values, e.g. three LEDs or one RGB LED.

## ROS messages

The following topics and messages are used by this driver.

| Type | Topic | Message |
|---|---|---|
| Subscriber | `camera/compressed` | []()|

## Hardware Interface

This package has been built and tested on a RPi 4B.  The interface used is as follows:

| GPIO Pin | Usage |
|---|---|
| 21 | Push button input |
| 22 | Status - Power On |
| 23 | Status - Ready |
| 24 | Status - Recording |

The hardware_interface.cpp file can be modified to suit your hardware.

## Build the ROS2 package

1. Clone this repo in the workspace `src` directory.
2. Build using `colcon build`.

## Acknowledgments

© 2024, University of Leeds.

The author, A. Blight, has asserted his moral rights.
