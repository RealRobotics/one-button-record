from pyfirmata import Arduino, util
import time

board = Arduino("/dev/ttyACM0")
while True:
    board.digital[13].write(0)
    time.sleep(2)
    print("on")
    board.digital[13].write(1)
    time.sleep(1)
    print("off")
