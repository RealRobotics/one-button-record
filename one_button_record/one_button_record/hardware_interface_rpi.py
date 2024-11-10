import logging
from GPIO import GPIO
from one_button_record.hardware_interface import HardwareInterface, LEDState


class HardwareInterfaceRPi(HardwareInterface):
    def __init__(self):
        logging.info("HIR::__init__")
        # GPIO Pin numbers.
        # TODO Change these as required.
        self._LED_POWER = 17
        self._LED_READY = 27
        self._LED_RECORDING = 22
        self.BUTTON_PIN = 23
        # Setup the LED pins
        GPIO.setup(self._LED_POWER, GPIO.OUT)
        GPIO.setup(self._LED_READY, GPIO.OUT)
        GPIO.setup(self._LED_RECORDING, GPIO.OUT)
        # Setup the button GPIO pin.
        GPIO.setup(self.BUTTON_PIN, GPIO.IN)

    # RPi implementation of LED control.
    def _set_all_off(self):
        logging.info("HIR::All off")

    def _set_power_on(self):
        GPIO.output(self._LED_POWER, GPIO.HIGH)
        GPIO.output(self._LED_READY, GPIO.LOW)
        GPIO.output(self._LED_RECORDING, GPIO.LOW)
        logging.info("HIR::Powering on")

    def _set_ready(self):
        GPIO.output(self._LED_POWER, GPIO.LoW)
        GPIO.output(self._LED_READY, GPIO.HIGH)
        GPIO.output(self._LED_RECORDING, GPIO.LOW)
        logging.info("HIR::Ready")

    def _set_recording(self):
        GPIO.output(self._LED_POWER, GPIO.LOW)
        GPIO.output(self._LED_READY, GPIO.LOW)
        GPIO.output(self._LED_RECORDING, GPIO.HIGH)
        logging.info("HIR::Recording")

    # RPi implementation for button press detection.
    def get_button_state(self) -> bool:
        # TODO May need to debounce the button.
        button_state = GPIO.input(self.BUTTON_PIN)
        return button_state == GPIO.HIGH
