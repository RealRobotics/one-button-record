# Design

## Review of existing code

As always, find out what is out there first.

This looks good: https://github.com/PINTO0309/mp42rosimg/tree/main.  Writes ROS2 images to file which is exactly what we need to do but we need to turn it on and off somehow.  How?

This project has a video writer: https://github.com/mlaiacker/rosbag2video.  I know my way around this code better.

This project uses the Raspberry Pi camera: https://github.com/pipebots/camera_ros/tree/main.  Note much use for what I'm doing but the composable node might be of use to use intra-process communication.

This is a ROS2 node that can be used : https://github.com/mlherd/ros2_pi_gpio

This file shows how to use GPIO pins on a Raspberry Pi: https://github.com/mlherd/ros2_pi_gpio/blob/master/pi_gpio/pi_gpio/pi_gpio_server.py

## Key problems to resolve

1. Subscribe to compressed image topic.  Standard.
2. Turn topic on and off.  When switched on, open file and write. When switched off, close file and then ignore any received messages.
3. Monitor GPIO for button press to turn topic on and off.
4. Control 3 GPIO output pins.
