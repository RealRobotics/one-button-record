import cv2
from cv_bridge import CvBridge

import datetime
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import time

# Select hardware inferface.
# import one_button_record.hardware_interface as hw_if
import one_button_record.hardware_interface_latte as hw_if
from one_button_record.hardware_interface import LEDState

from sensor_msgs.msg import CameraInfo, CompressedImage


class ImageWriter:
    def __init__(self):
        # Use [0,0] as a not set value for the frame size.
        self.frame_size = [0, 0]  # : Tuple[int, int],
        # The camera says it output bgr8.
        self.msg_image_format = "bgr8"
        self.video_writer_fps = 30.0  #: float,
        self.output_file_name = "test.mp4"  #: str,
        self.output_codec_type = "mp4v"  # : str
        self._video_writer = None
        self._cv_bridge = CvBridge()

    def open(self, filename):
        self._video_writer = cv2.VideoWriter(
            filename,
            fourcc=cv2.VideoWriter_fourcc(*self.output_codec_type),
            fps=self.video_writer_fps,
            frameSize=self.frame_size,
        )

    def close(self):
        self._video_writer.release()

    def write(self, msg):
        if self.frame_size != [0, 0]:
            if self._video_writer is not None:
                mat = self._cv_bridge.compressed_imgmsg_to_cv2(
                    msg, self.msg_image_format
                )
                self._video_writer.write(mat)


# Topics when running laptop camera.
# /camera_info
# /image_raw
# /image_raw/compressed
# /image_raw/compressedDepth
# /image_raw/theora
# /image_raw/zstd


class ImageSubscriberNode(Node):
    def __init__(self, hardware_interface):
        super().__init__("image_subscriber")
        self._ready = False
        self._recording = False
        # FIXME hard coded file name.
        self._filename = "test.mp4"
        self._hardware_interface = hardware_interface
        self._image_writer = ImageWriter()
        self._image_subscriber = self.create_subscription(
            CompressedImage, "image_raw/compressed", self.image_callback, 10
        )
        self._camera_info_subscriber = self.create_subscription(
            CameraInfo, "camera_info", self.camera_info_callback, 10
        )
        # Create a timer that fires at 20Hz
        timer_period = 0.05
        self._timer = self.create_timer(timer_period, self.timer_callback)
        # Blink interval vars.
        self._blink_next_t = time.time()
        self._BLINK_INTERVAL_S = 0.25  # Seconds

    def timer_callback(self):
        # self.get_logger().info("timer has fired")
        # Poll the state of the push button.
        button_state = self._hardware_interface.get_button_state()
        if button_state:
            if self._recording:
                self.stop_recording()
            else:
                self.start_recording(self._filename)
        # Update LEDs.
        if self._blink_next_t < time.time():
            self._hardware_interface.blink()
            self._blink_next_t = time.time() + self._BLINK_INTERVAL_S
        if self._recording:
            self._hardware_interface.set_led_state(LEDState.RECORDING)
        elif self._ready:
            self._hardware_interface.set_led_state(LEDState.READY)
        else:
            self._hardware_interface.set_led_state(LEDState.POWER_ON)

    def image_callback(self, msg):
        # CompressedImage.msg format.
        # Header header    # Header timestamp should be acquisition time of image
        # string format    # Acceptable values: jpeg, png
        # uint8[] data     # Compressed image buffer
        self.get_logger().info('Frame format: "%s"' % msg.format)
        if self._recording:
            self._image_writer.write(msg)

    def camera_info_callback(self, msg):
        # CameraInfo.msg format.
        # We are only interested in the height and width of the image.
        # Header header
        # uint32 height
        # uint32 width
        self.get_logger().info("Camera info callback.")
        self._image_writer.frame_size = [msg.width, msg.height]
        self._ready = True

    def start_recording(self, filename):
        if self._image_writer.frame_size != [0, 0]:
            self._image_writer.open(filename)
            self._recording = True
            self._filename = filename
            self.get_logger().info('Started recording to file: "%s"' % filename)
        else:
            self.get_logger().error(
                "Cannot start recording. Camera info not available."
            )

    def stop_recording(self):
        self._image_writer.close()
        self._recording = False
        self.get_logger().info('Stopped recording to file: "%s"' % self._filename)
        self._hardware_interface.set_led_state(LEDState.READY)


def main(args=None):
    print("Setting up hardware interface.")
    # hw = hw_if.HardwareInterface()
    hw = hw_if.HardwareInterfaceLatte()
    hw.set_led_state(LEDState.POWER_ON)
    print("Subscribing to topics.")
    rclpy.init(args=args)
    image_subscriber = ImageSubscriberNode(hw)
    try:
        rclpy.spin(image_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    image_subscriber.destroy_node()
    # FIXME(AJB) Generates exception after Ctrl-C.  Looks like it is a bug in rclpy.
    rclpy.shutdown()
    print("Shutdown.")


if __name__ == "__main__":
    main()
