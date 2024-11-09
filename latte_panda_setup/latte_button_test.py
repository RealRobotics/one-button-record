import pyfirmata2
import time

print("Getting ready...")


def toggle_blink_led():
    global blink_state
    blink_state = not blink_state
    board.digital[LED_RED].write(blink_state)

def button_callback(value):
    if value:
        board.digital[LED_READY].write(True)
        board.digital[LED_RECORDING].write(False)
        print("Released")
    else:
        board.digital[LED_READY].write(False)
        board.digital[LED_RECORDING].write(True)
        print("Pressed")

PORT = pyfirmata2.Arduino.AUTODETECT
board = pyfirmata2.Arduino(PORT)
blink_state = False
LED_RED = 13  # The red LED
LED_READY = 9
LED_RECORDING = 10
# Button input pin
# Turn on sampling at default 19ms intervals.
board.samplingOn()
# Set up pin as digital input with pull up resistor.
input_pin = board.get_pin("d:11:u")
# Register callback and enable.
input_pin.register_callback(button_callback)
input_pin.enable_reporting()

# Loop forever blinking LED
while True:
    toggle_blink_led()
    time.sleep(1)
    toggle_blink_led()
    time.sleep(1)

