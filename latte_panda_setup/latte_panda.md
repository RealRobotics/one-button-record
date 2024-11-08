# LattePanda Setup

How the LattePanda board was setup for the One Button Record package.

## Setting up the Arduino

The first thing was to program the Arduino Mega chip so we could use the GPIO pins.  [This guide](http://docs.lattepanda.com/content/alpha_edition/python_arduino_bridge/#step-2) covers most of the steps but for Windows so you follow the general process as we are using Ubuntu.  These are things that needed to be  done differently.

The latest Arduino IDE was downloaded and unzipped it here: `~/software/arduino-ide_2.3.3_Linux_64bit`.  I ran the executable `arduino-ide`  from the command and it failed due to a known bug. The fix is documented in the output and involves changing the ownership of the `chrome-sandbox` to `root` and setting the permissions to `4755`.  Once that was done, the IDE runs.

The user needs to be added to the `dialout ` group to allow use of the `tty` nodes which are used by the Arduino IDE when up loading programs. I used the command:

```bash
sudo adduser $USER dialout
```

and then rebooted!  I know that you should just be able to log out and in again, but only a reboot worked.  

At this point, I was able to download a blink program and flash the red LED on the board.

## Setting up the Firmata library

The documentation refers to `pyFirmata` but this is no longer supported and has been replaced by `pyFirmata2`.

Installing the pyFirmata2 library was straightforward except for the overriding of permissions.  Ubuntu tries to insist that you only install Python packages using `pip` in a virtual environment (VENV).  However, as this is a single board computer doing one thing, we can ignore the warnings.  I installed the library like this:

```bash
sudo pip install --break-system-packages pyFirmata2
```

and tested the install as described in the LattePanda docs.  It worked so copied their demo program, changed the port and ran it and it works.

__NOTE: When starting this program, there is a delay of a about 7 seconds before the LED starts to blink.__

### Notes on Firmata install

The code failed first time and that is when I found out about Firmata2. The error message is below:

```text
$ python3 arduino_blink.py 
Traceback (most recent call last):
  File "/home/pharos/ws/src/one-button-record/latte-panda-setup/arduino_blink.py", line 4, in <module>
    board = Arduino("/dev/ttyACM0")
            ^^^^^^^^^^^^^^^^^^^^^^^
  File "/usr/local/lib/python3.12/dist-packages/pyfirmata/__init__.py", line 19, in __init__
    super(Arduino, self).__init__(*args, **kwargs)
  File "/usr/local/lib/python3.12/dist-packages/pyfirmata/pyfirmata.py", line 101, in __init__
    self.setup_layout(layout)
  File "/usr/local/lib/python3.12/dist-packages/pyfirmata/pyfirmata.py", line 157, in setup_layout
    self._set_default_handlers()
  File "/usr/local/lib/python3.12/dist-packages/pyfirmata/pyfirmata.py", line 161, in _set_default_handlers
    self.add_cmd_handler(ANALOG_MESSAGE, self._handle_analog_message)
  File "/usr/local/lib/python3.12/dist-packages/pyfirmata/pyfirmata.py", line 185, in add_cmd_handler
    len_args = len(inspect.getargspec(func)[0])
                   ^^^^^^^^^^^^^^^^^^
AttributeError: module 'inspect' has no attribute 'getargspec'. Did you mean: 'getargs'?
```

A quick search later and I find [this article](https://stackoverflow.com/questions/74585622/pyfirmata-gives-error-module-inspect-has-no-attribute-getargspec) that says that `pyFirmata` is no longer being supported but `pyFirmata2` is.  So time to install that and see what happens.  