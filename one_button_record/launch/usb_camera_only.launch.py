# MIT License
#
# Copyright (c) 2024 University of Leeds
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path  # noqa: E402


def generate_launch_description():
    ld = LaunchDescription()

    dir_path = os.path.dirname(os.path.realpath(__file__))
    param_path = Path(dir_path, 'config', 'usb_camera_params.yaml')

    camera_node = Node(
        # ros2 run usb_cam usb_cam_node_exe
        package="usb_cam",
        executable="usb_cam_node_exe",
        parameters=[param_path],
    )

    ld.add_action(camera_node)

    return ld
