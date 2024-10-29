import time
import one_button_record.hardware_interface


def main():
    print("Hi from one_button_record.")
    # Create an instance of the HardwareInterface class
    # with pin_id = 18 and type = "out"
    hw = one_button_record.hardware_interface.HardwareInterface()
    # Simulate booting up the hardware.
    hw.set_led_state(one_button_record.hardware_interface.LEDState.POWER_ON)
    time.sleep(1)
    hw.set_led_state(one_button_record.hardware_interface.LEDState.READY)
    # Simulate the button press.
    recording = False
    while True:
        if hw._get_button_state():
            if recording:
                recording = False
            else:
                recording = True
            if recording:
                hw.set_led_state(
                    one_button_record.hardware_interface.LEDState.RECORDING
                )
            else:
                hw.set_led_state(one_button_record.hardware_interface.LEDState.READY)


if __name__ == "__main__":
    main()
