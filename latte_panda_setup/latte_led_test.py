import pyfirmata2
import time

print("Getting ready...")

blink_state = False

PORT = pyfirmata2.Arduino.AUTODETECT
board = pyfirmata2.Arduino(PORT)
# board = pyfirmata2.Arduino("/dev/ttyACM0")
# GPIO Pin numbers.
LED_RED = 13  # The red LED
LED_POWER = 8
LED_READY = 9
LED_RECORDING = 10


def toggle_blink_led():
    global blink_state
    blink_state = not blink_state
    board.digital[LED_RED].write(blink_state)


while True:
    board.digital[LED_POWER].write(True)
    board.digital[LED_READY].write(False)
    board.digital[LED_RECORDING].write(False)
    print("ON")
    toggle_blink_led()
    time.sleep(1)
    board.digital[LED_POWER].write(True)
    board.digital[LED_READY].write(True)
    board.digital[LED_RECORDING].write(False)
    print("READY")
    toggle_blink_led()
    time.sleep(1)
    board.digital[LED_POWER].write(True)
    board.digital[LED_READY].write(True)
    board.digital[LED_RECORDING].write(True)
    print("RECORDING")
    toggle_blink_led()
    time.sleep(1)
    board.digital[LED_POWER].write(False)
    board.digital[LED_READY].write(False)
    board.digital[LED_RECORDING].write(False)
    print("OFF")
    toggle_blink_led()
    time.sleep(1)
