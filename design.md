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

## Implementation

### Laptop

The first cut of the code was developed on my laptop as it is way faster than the single board computer that I will be using.  Wrote the code for everything including the untested GPIO code for a Raspberry Pi.  Got it working using the package `usb_cam` to access the built-in camera.

### LattePanda

The LattePanda uses the Arduino chip for all GPIO access, so we need an Arduino program to handle the button press input and the LED outputs.  That means we need to program the Arduino and then use some sort of interface to communicate between the main CPU and the Arduino.

Fortunately, LattePanda have thought about this and use an Arduino library called `Firmata` and a corresponding Python library called `pyFirmata`.  This allows us to write Python code on the main CPU that uses the Arduino GPIO pins in a very simple way.

Details on setting up the LattePanda board can be found [here](latte-panda-setup/latte-panda.md).

The LattePanda implementation of the `HardwareInterface` class implements the LED functions and the `get_button_press` function.  This code was prototyped in the `latte_button_test.py` script to limit the scope of the problem and then the working code was added to the `HardwareInterfaceLatte` class.

### One button record

Now the hardware interface was doing what I wanted, I started testing the top level code.  There were numerous small issues with getting the camera working correctly, but a  basic working set of values were found and the launch file `launch/usb_camera_only.launch.py` can be used to start the camera successfully.

Running the node had many small issues with logging and getting the image parameters correct.  Once these were fixed, the node runs well with one issue: when the button is pressed and held down for mor than about 0.5 seconds, the recording starts and then immediately stops.  What needs to happen is when the button is pressed and held down, the recording starts.  The recording should stop only after the button has been released and pressed again.

After a bit of thinking, I realised that I needed 5 states: `power_on`, `ready`, `starting_recording`, `recording`, `stopping_recording`.  The `starting_recording` and `stopping_recording` states are transitioned into when the button is pressed and exited when the button is released.

When I implemented this, it ended up as more of a sub-state just related to the button state.  Adding the variable `self._button_was_pressed` and related logic solved the problem very neatly.

The next thing was to add a timestamp to the video filename to stop overwriting of previous files.  Very simple as I just used the function `datetime.datetime.now().isoformat()` for the filename and appended the `.mp4`. Or so I thought until I tested it when I got this error:

```text
[ WARN:0@10.004] global ./modules/videoio/src/cap_gstreamer.cpp (2180) open OpenCV | GStreamer warning: cannot link elements
[INFO] [1731231381.694751795] [image_subscriber]: Started recording to file: "2024-11-10T09:36:21.mp4"
```

and this error stopped the video being created.  I suspected that either the hyphens or the colons are causing problems inside GStreamer, so I changed the datetime function to this `                 datetime.datetime.now().strftime("%Y%m%dT%H%M%S")` which worked nicely.  At this stage, I also decided that the videos should be put in the `~/Videos` directory as they were cluttering up the workspace.  This caused the same old problem:

```text
[ WARN:0@8.901] global ./modules/videoio/src/cap_gstreamer.cpp (2180) open OpenCV | GStreamer warning: cannot link elements
[INFO] [1731232285.871292340] [image_subscriber]: Started recording to file: "~/Videos/20241110T095125.mp4"
```

So took that change out for now.

Worked out how to get a clean shutdown, don't call `rclpy.shutdown()`.  Added LED state `ALL_OFF` and turn all LEDs off when stopping the program.

Sorted out use of camera params file (took a while) and can save at full HD resolution.

Added timeout on `camera_info` topic so `READY` LED turns on and off as the camera is run and stopped.

## TODO

* Write videos to `~/Videos` directory.

