/**
 * File: controller.ino
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file serves as the base sketch for theBasicBot's ESP32-based controller.
 */

/* Constants ------------------------------------------------------------------------------------ */

// Left button cluster
static constexpr int kPinButtonLeft_Up = 21;
static constexpr int kPinButtonLeft_Right = 19;
static constexpr int kPinButtonLeft_Down = 18;
static constexpr int kPinButtonLeft_Left = 5;

// Right button cluster
static constexpr int kPinButtonRight_Up = 17;
static constexpr int kPinButtonRight_Right = 16;
static constexpr int kPinButtonRight_Down = 4;
static constexpr int kPinButtonRight_Left = 0;

// Shoulder buttons
static constexpr int kPinButtonShoulder_Left = 25;
static constexpr int kPinButtonShoulder_Right = 13;

// Start and select
static constexpr int kPinButtonStart = 15;
static constexpr int kPinButtonSelect = 2;

// Left Joystick
static constexpr int kPinJoystickLeft_X = 26;
static constexpr int kPinJoystickLeft_Y = 27;
static constexpr int kPinJoystickLeft_Button = 32;

// Right Joystick
static constexpr int kPinJoystickRight_X = 14;
static constexpr int kPinJoystickRight_Y = 12;
static constexpr int kPinJoystickRight_Button = 33;

// Status LED
static constexpr int kPinLed_Red = 23;
static constexpr int kPinLed_Green = 22;

// BLE adapter programming port
static constexpr int kPinEspTx_to_BleRx = 34;
static constexpr int kPinEspRx_to_BleTx = 35;

/* Setup and Loop ------------------------------------------------------------------------------- */

void setup() {
  // Left button cluster
  pinMode(kPinButtonLeft_Up, INPUT);
  pinMode(kPinButtonLeft_Right, INPUT);
  pinMode(kPinButtonLeft_Down, INPUT);
  pinMode(kPinButtonLeft_Left, INPUT);

  // Right button cluster
  pinMode(kPinButtonRight_Up, INPUT);
  pinMode(kPinButtonRight_Right, INPUT);
  pinMode(kPinButtonRight_Down, INPUT);
  pinMode(kPinButtonRight_Left, INPUT);

  // Shoulder buttons
  pinMode(kPinButtonShoulder_Left, INPUT);
  pinMode(kPinButtonShoulder_Right, INPUT);

  // Start and select
  pinMode(kPinButtonStart, INPUT);
  pinMode(kPinButtonSelect, INPUT);

  // Left Joystick
  pinMode(kPinJoystickLeft_X, INPUT);
  pinMode(kPinJoystickLeft_Y, INPUT);
  pinMode(kPinJoystickLeft_Button, INPUT);

  // Right Joystick
  pinMode(kPinJoystickRight_X, INPUT);
  pinMode(kPinJoystickRight_Y, INPUT);
  pinMode(kPinJoystickRight_Button, INPUT);

  // Status LED
  pinMode(kPinLed_Red, OUTPUT);
  pinMode(kPinLed_Green, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
}