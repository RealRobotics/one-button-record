import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import time
import one_button_record.hardware_interface as hw_if
#import one_button_record.hardware_interface_rpi as hw_if

from sensor_msgs.msg import CompressedImage


class ImageWriter():
    def __init__(self):
        pass

    def open(self, filename):
        pass

    def close(self):
        pass

    def write(self, image):
        pass


class ImageSubscriberNode(Node):
    def __init__(self, hardware_interface):
        super().__init__("image_subscriber")
        self._recording = False
        # FIXME hard coded file name.
        self._filename = "test.mp4"
        self._hardware_interface = hardware_interface
        self._image_writer = ImageWriter()
        self._subscription = self.create_subscription(
            CompressedImage, "camera/compressed_image", self.listener_callback, 10
        )
        self._subscription  # prevent unused variable warning
        # Create a timer that fires at 20Hz
        timer_period = 0.05
        self._timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("timer has fired")
        # Poll the state of the push button.
        button_state = self._hardware_interface.get_button_state()
        if button_state:
            if self._recording:
                self.stop_recording()
            else:
                self.start_recording(self._filename)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if self._recording:
            self._image_writer.write(msg.data)

    def start_recording(self, filename):
        self._image_writer.open(filename)
        self._recording = True
        self._filename = filename
        print("Started recording to file: ", filename)

    def stop_recording(self):
        self._image_writer.close()
        self._recording = False
        print("Stopped recording to file: ", self._filename)


def main(args=None):
    print("Setting up hardware interface.")
    hw = hw_if.HardwareInterface()
    # hw = hw_if.HardwareInterfaceRPi()
    print("Subscribing to image topic.")
    rclpy.init(args=args)
    image_subscriber = ImageSubscriberNode(hw)
    try:
        rclpy.spin(image_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    image_subscriber.destroy_node()
    # Fix me.
    # Generates exception after Ctrl-C.
    # Looks like it is a bug in rclpy.
    rclpy.shutdown()


if __name__ == "__main__":
    main()
