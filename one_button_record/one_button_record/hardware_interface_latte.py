from pyfirmata2 import pyfirmata2, Arduino
from one_button_record.hardware_interface import HardwareInterface, LEDState


class HardwareInterfaceLatte(HardwareInterface):
    """Hardware interface class for LattePanda 3 Delta.
    Implemented using the pyFirmata2 library.
    """

    def __init__(self):
        PORT = pyfirmata2.Arduino.AUTODETECT
        self._board = pyfirmata2.Arduino(PORT)
        # GPIO Pin numbers.
        self._LED_RED = 13  # The red LED
        self._LED_POWER = 8
        self._LED_READY = 9
        self._LED_RECORDING = 10
        # Turn the Arduino LED on.
        self._board.digital[self._LED_RED].write(True)
        # Setup the other LED pins
        self._board.digital[self._LED_POWER].write(False)
        self._board.digital[self._LED_READY].write(False)
        self._board.digital[self._LED_RECORDING].write(False)
        # Setup the input pin.
        self._input_pin_last_value = False
        self._board.samplingOn()
        # The interface uses a string so we can't use a variable.
        input_pin = self._board.get_pin("d:11:u")
        input_pin.register_callback(self._input_pin_callback)
        input_pin.enable_reporting()

    def _input_pin_callback(self, value):
        # Needs hardware debounce.
        self._input_pin_last_value = value

    def get_button_state(self) -> bool:
        return self._input_pin_last_value

    def set_led_state(self, state: LEDState):
        if state == LEDState.POWER_ON:
            self._board.digital[self._LED_POWER].write(True)
            self._board.digital[self._LED_READY].write(False)
            self._board.digital[self._LED_RECORDING].write(False)
        elif state == LEDState.READY:
            self._board.digital[self._LED_POWER].write(True)
            self._board.digital[self._LED_READY].write(True)
            self._board.digital[self._LED_RECORDING].write(False)
        elif state == LEDState.RECORDING:
            self._board.digital[self._LED_POWER].write(True)
            self._board.digital[self._LED_READY].write(True)
            self._board.digital[self._LED_RECORDING].write(True)
        else:
            print("ERROR")
