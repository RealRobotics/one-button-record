# One Button Record

This repo contains code to start and stop recording of video on a ROS2 robot.  The robot must be fitted with a push button switch and a means of displaying three status values, e.g. three LEDs or one RGB LED.

The design of this package is detailed [here](design.md).

## ROS messages

The following topics and messages are used by this driver.

| Type | Topic | Message |
|---|---|---|
| Subscriber | `camera/compressed` | [sensor_msgs/msg/CompressedImage Message](https://docs.ros2.org/latest/api/sensor_msgs/msg/CompressedImage.html)|

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
3. Run using `ros2 run one_button_record one_button_record`.
4. To test the hardware interface only, use `ros2 run one_button_record hw_test`.

## Troubleshooting

I had issues with Python not being able to import the `cv2` module.  This was caused by PlatformIO setting the `PATH` environment variable to point that the virtual environment (VENV) that it uses before the installed versions, in this case `/home/andy/.platformio/penv/bin`.  I fixed this and the scripts worked.

## Acknowledgments

© 2024, University of Leeds.

The author, A. Blight, has asserted his moral rights.
