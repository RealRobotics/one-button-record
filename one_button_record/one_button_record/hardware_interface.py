from enum import Enum

# Define the LED states
LEDState = Enum("LEDState", ["POWER_ON", "READY", "RECORDING", "ERROR"])


class HardwareInterface:
    def __init__(self):
        pass
        # Setup the LED and button GPIO pins.
        # GPIO.setup(led_pin, GPIO.OUT)
        # GPIO.setup(button_pin, GPIO.IN)

    def _set_power_on(self):
        print("Powering on...")

    def _set_ready(self):
        print("Ready...")

    def _set_recording(self):
        print("Recording...")

    def get_button_state(self) -> bool:
        # Simulate the button state.
        if input("Press Enter to simulate button press: "):
            return True
        return False

    def set_led_state(self, state: LEDState):
        if state == LEDState.POWER_ON:
            self._set_power_on()
        elif state == LEDState.READY:
            self._set_ready()
        elif state == LEDState.RECORDING:
            self._set_recording()
        else:
            print("ERROR")
