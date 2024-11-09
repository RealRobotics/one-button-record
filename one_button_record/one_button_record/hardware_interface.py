from enum import Enum

# Define the LED states
LEDState = Enum("LEDState", ["POWER_ON", "READY", "RECORDING", "ERROR"])


class HardwareInterface:
    def __init__(self):
        # No hardware to set up!
        print("HI::__init__")

    def _set_power_on(self):
        print("HI::Powering on...")

    def _set_ready(self):
        print("HI::Ready...")

    def _set_recording(self):
        print("HI::Recording...")

    def get_button_state(self) -> bool:
        print("HI::gbs")
        # Simulate the button state.
        if input("Press Enter to simulate button press: "):
            return True
        return False

    def set_led_state(self, state: LEDState):
        print("HI::sls")
        if state == LEDState.POWER_ON:
            self._set_power_on()
        elif state == LEDState.READY:
            self._set_ready()
        elif state == LEDState.RECORDING:
            self._set_recording()
        else:
            print("ERROR")

    def blink(self):
        print("HI::Blink")
