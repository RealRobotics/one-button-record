import time
# import one_button_record.hardware_interface as hw_if
import one_button_record.hardware_interface_latte as hw_if


def boot_up(hw):
    # Simulate booting up the hardware.
    hw.set_led_state(hw_if.LEDState.POWER_ON)
    time.sleep(1)
    hw.set_led_state(hw_if.LEDState.READY)


def test_hw(hw):
    recording = False
    completed = False
    while not completed:
        if hw.get_button_state():
            if recording:
                recording = False
            else:
                recording = True
            if recording:
                hw.set_led_state(hw_if.LEDState.RECORDING)
            else:
                hw.set_led_state(hw_if.LEDState.READY)
                completed = True


def main(args=None):
    print("Starting hardware tests for one_button_record.")
    hw = hw_if.HardwareInterface()
    # hw = hw_if.HardwareInterfaceRPi()
    boot_up(hw)
    test_hw(hw)
    print("Tests complete.")


if __name__ == "__main__":
    main()
