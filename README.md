# BleebleTech Controllers

[toc]

## Introduction

The [BleebleTech Basic and Advanced Controllers](https://bleebletech.ca) are made to be simple, hackable tools for you to use in any electronics project that requires button presses or joystick movement from a user. They were originally conceived of and designed for use with [The Basic Bot](https://thebasicbot.com) robotics platform and education curriculum, however are absolutely usable for any other BLE project!

## Setup

To program either the **BleebleTech Basic/Advanced Controllers**, or **The Basic Bot (tBB) robots**, you'll need the [Arduino IDE](https://www.arduino.cc/en/software).

### Controller-Specific Setup

Once the Arduino IDE is installed, you'll need to install the following board package by searching for it in the the "Boards Manager" on the left of the IDE:

- ["esp32" by Espressif Systems](https://github.com/espressif/arduino-esp32)
  - This allows the Arduino IDE to properly handle the ESP32 microcontroller used in the BleebleTech controllers instead of a typical Arduino Uno or similar board

Next, you'll need to search for and install the following libraries in the "Library Manager" on the left of the IDE:

- ["EspSoftwareSerial" by Dirk Kaar, Peter Lerup](https://github.com/plerup/espsoftwareserial/)
  - This provides a serial data protocol (a way to send data over some wires to other devices) that can be used on any pins, just like the standard Arduino SoftwareSerial library, but this one is written specifically for the ESP32 microcontroller
- ["Preferences" by Volodymyr Shymanskyy](https://github.com/vshymanskyy/Preferences)
  - This allows some data to be stored and remembered even after the device restarts or loses power (hint: remembering which BLE device to connect to)

**Note:** The online Arduino IDE is not officially supported by BleebleTech for programming the BleebleTech controllers, since it doesn't play nicely with libraries and the ESP32 microcontroller, but it can be used with some hacky workarounds if necessary.
Keep in mind that controllers should only need to be programmed once after being assembled/received, unless you modify them (which you should absolutely try!)
Contact us via [the BleebleTech website](https://bleebletech.ca/pages/contact) if you require this, and we'll do our best to help you out.

## Programming the Controllers

Programming the BleebleTech controllers is super easy:

1. Plug the controller into your computer with a Micro-USB cable
2. Open the `BleebleTech_Controller/BleebleTech_Controller.ino` file from this repository, which should open in the Arduino IDE
3. At the top of the Arduino IDE, click the "Select Board" dropdown, then select the COM port shown
  a. The board type may be shown as "Unknown", this is okay
  b. If you see multiple COM ports listed, take note of all the list items, unplug the controller, look at the list again, and take note of which item disappeared. This is the COM port assigned to the controller, so plug the controller back in, then select the now re-added COM port!
4. Click the "Upload" arrow just to the left of the board selector.
  a. This will take longer than usual the first time around, and may be a few minutes on any modern computer. On some very old and under-powered laptops, this has been known to take up to *20-25 minutes*, so please be patient the first time if your computer matches this description. This time is unfortunately not something we can control or improve, and is a limitation of the computing power and Arduino IDE's compiler.

Contact us via [the BleebleTech website](https://bleebletech.ca/pages/contact) if you have any troubles that you can't seem to figure out on your own, and we'll do our best to help you out.
