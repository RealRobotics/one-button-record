import time
from one_button_record.hardware_interface import LEDState
import one_button_record.hardware_interface_latte as hw_if


def boot_up(hw):
    # Simulate booting up the hardware.
    hw.set_led_state(LEDState.POWER_ON)
    time.sleep(1)
    hw.set_led_state(LEDState.READY)


def test_hw(hw):
    recording = False
    completed = False
    while not completed:
        hw.blink()
        time.sleep(0.2)
        button_state = hw.get_button_state()
        if button_state is True:
            if recording:
                recording = False
            else:
                recording = True
            if recording:
                hw.set_led_state(LEDState.RECORDING)
            else:
                hw.set_led_state(LEDState.READY)
                completed = True


def main(args=None):
    print("Starting hardware tests for one_button_record.")
    # hw = hw_if.HardwareInterface()
    hw = hw_if.HardwareInterfaceLatte()
    boot_up(hw)
    test_hw(hw)
    print("Tests complete.")


if __name__ == "__main__":
    main()
