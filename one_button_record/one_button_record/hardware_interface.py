import logging
from enum import Enum

# Define the LED states
LEDState = Enum("LEDState", ["POWER_ON", "READY", "RECORDING", "ERROR"])


class HardwareInterface:
    def __init__(self):
        # logging.basicConfig(level=logging.INFO)
        logging.info("HI::__init__")

    def _set_power_on(self):
        logging.info("HI::Powering on...")

    def _set_ready(self):
        logging.info("HI::Ready...")

    def _set_recording(self):
        logging.info("HI::Recording...")

    def get_button_state(self) -> bool:
        logging.info("HI::gbs")
        # Simulate the button state.
        if input("Press Enter to simulate button press: "):
            return True
        return False

    def set_led_state(self, state: LEDState):
        logging.info("HI::sls")
        if state == LEDState.POWER_ON:
            self._set_power_on()
        elif state == LEDState.READY:
            self._set_ready()
        elif state == LEDState.RECORDING:
            self._set_recording()
        else:
            logging.info("ERROR")

    def blink(self):
        logging.info("HI::Blink")
