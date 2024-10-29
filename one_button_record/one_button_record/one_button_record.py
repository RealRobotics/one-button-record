import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import time
import one_button_record.hardware_interface as hw_if

from sensor_msgs.msg import CompressedImage


class ImageSubscriberNode(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.subscription = self.create_subscription(
            CompressedImage, "camera/compressed_image", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def boot_up(hw):
    # Simulate booting up the hardware.
    hw.set_led_state(hw_if.LEDState.POWER_ON)
    time.sleep(1)
    hw.set_led_state(hw_if.LEDState.READY)


def test_hw(hw):
    recording = False
    completed = False
    while not completed:
        if hw.get_button_state():
            if recording:
                recording = False
            else:
                recording = True
            if recording:
                hw.set_led_state(
                    hw_if.LEDState.RECORDING
                )
            else:
                hw.set_led_state(hw_if.LEDState.READY)
                completed = True


def main(args=None):
    print("Hi from one_button_record.")
    hw = hw_if.HardwareInterface()
    boot_up(hw)
    test_hw(hw)

    print("Subscribing to image topic.")
    rclpy.init(args=args)
    image_subscriber = ImageSubscriberNode()
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
