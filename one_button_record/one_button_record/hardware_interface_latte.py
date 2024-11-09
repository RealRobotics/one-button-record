import pyfirmata2
from one_button_record.hardware_interface import HardwareInterface, LEDState

class HardwareInterfaceLatte(HardwareInterface):
    """Hardware interface class for LattePanda 3 Delta.
    Implemented using the pyFirmata2 library.
    """

    def __init__(self):
        print("HIL::__init__")
        PORT = pyfirmata2.Arduino.AUTODETECT
        self._board = pyfirmata2.Arduino(PORT)
        # GPIO Pin numbers.
        self._LED_RED = 13  # The red LED
        self._LED_POWER = 8
        self._LED_READY = 9
        self._LED_RECORDING = 10
        # Turn the Arduino LED on.
        self._board.digital[self._LED_RED].write(True)
        self._blink_state = True
        # Setup the other LED pins
        self._board.digital[self._LED_POWER].write(False)
        self._board.digital[self._LED_READY].write(False)
        self._board.digital[self._LED_RECORDING].write(False)
        # Button input pin
        # Turn on sampling at default 19ms intervals.
        self._board.samplingOn()
        # Set up pin as digital input with pull up resistor.
        input_pin = self._board.get_pin("d:11:u")
        # Register callback and enable.
        input_pin.register_callback(self._input_pin_callback)
        input_pin.enable_reporting()

    def _input_pin_callback(self, value):
        # Needs hardware debounce.
        if value:
            print("HIL::Released")
        else:
            print("HIL::Pressed")
        self._input_pin_last_value = not value

    # These are called by the base class.
    def _set_power_on(self):
        self._board.digital[self._LED_POWER].write(True)
        self._board.digital[self._LED_READY].write(False)
        self._board.digital[self._LED_RECORDING].write(False)
        print("HIL::Power on...")

    def _set_ready(self):
        self._board.digital[self._LED_POWER].write(True)
        self._board.digital[self._LED_READY].write(True)
        self._board.digital[self._LED_RECORDING].write(False)
        print("HIL::Ready...")

    def _set_recording(self):
        self._board.digital[self._LED_POWER].write(True)
        self._board.digital[self._LED_READY].write(True)
        self._board.digital[self._LED_RECORDING].write(True)
        print("HIL::Recording...")

    # This overrides the based class implementation.
    def get_button_state(self) -> bool:
        print("HIL::gbs")
        return self._input_pin_last_value

    def blink(self):
        print("HIL::Blink")
        self._blink_state = not self._blink_state
        self._board.digital[self._LED_RED].write(self._blink_state)

