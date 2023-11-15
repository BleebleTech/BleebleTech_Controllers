/**
 * File: robot.ino
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file serves as an example for robots controlled by theBasicBot's ESP32-based controller.
 */

/* Includes ------------------------------------------------------------------------------------- */

#include <Servo.h>

/* Constants ------------------------------------------------------------------------------------ */

// Motor control pins connected to H-Bridge motor driver
static constexpr int kRightWheel_Backwards = 3;
static constexpr int kRightWheel_Forwards = 5;
static constexpr int kLeftWheel_Forwards = 6;
static constexpr int kLeftWheel_Backwards = 11;

// Joystick parsing configuration
static constexpr uint8_t kJoystick_Middle = 128;
static constexpr uint8_t kJoystick_Deadzone = 16;
static constexpr uint8_t kJoystick_Maximum = 254;

// Motor control configuration
static constexpr uint8_t kMotorMaximum = 254;
// Adjust either of these down (NOT ABOVE 255) if one motor is faster than the other
static constexpr uint8_t kMotorMaximum_Left = 255;
static constexpr uint8_t kMotorMaximum_Right = 255;

// Servo configuration
static constexpr int kPinArmServo = 9;
static constexpr int kPinGateServo = 10;

static constexpr uint8_t kArmServoMin = 60;
static constexpr uint8_t kArmServoMax = 120;
static constexpr uint8_t kGateServoMin = 0;
static constexpr uint8_t kGateServoMax = 90;
static constexpr uint8_t kRightPaddleServoMin = 90;
static constexpr uint8_t kRightPaddleServoMax = 180;

static constexpr uint16_t kArmServoSpeed = 1;
static constexpr uint16_t kPaddleServoSpeed = 6;

// Message decoding configuration
static constexpr int kMessageSize_B = 7;

/* Variables ------------------------------------------------------------------------------------ */

static uint8_t rxBuffer[kMessageSize_B] = {};
static uint8_t rxCache[kMessageSize_B] = {};
static size_t numRxBytes = 0;

static Servo armServo;
static Servo gateServo;

static uint8_t armServoPos = 90;
static uint8_t gateServoPos = 90;
static uint8_t rightPaddleServoPos = 90;

static bool firstMsgRx = false;

/* Functions ------------------------------------------------------------------------------------ */
static void parseBleMessage(const uint8_t* const aMsg) {
  // LEAVE THIS COMMENTED OUT IF YOU'RE NOT USING IT!
  // Print out the buffer for the user to see
  /*
  char s[48] = {0};
  snprintf(s, 47, "RX: [0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x]\n", aMsg[0],
           aMsg[1], aMsg[2], aMsg[3], aMsg[4], aMsg[5], aMsg[6]);
  Serial.println(s);
  */
  uint8_t c = 0;

  // Servos
  c = aMsg[2];
  if ((c & 0x02) && armServoPos < kArmServoMax) {
    armServoPos += kArmServoSpeed;
  } else if ((c & 0x04) && armServoPos > kArmServoMin) {
    armServoPos -= kArmServoSpeed;
  }

  armServo.write(armServoPos);

  if ((c & 0x01) && gateServoPos < kGateServoMax) {
    gateServoPos += kPaddleServoSpeed;
  } else if ((c & 0x08) && gateServoPos > kGateServoMin) {
    gateServoPos -= kPaddleServoSpeed;
  }

  gateServo.write(gateServoPos);

  // Left motor
  c = aMsg[4];
  if (c >= (kJoystick_Middle + kJoystick_Deadzone)) {
    analogWrite(kLeftWheel_Forwards, map(c, kJoystick_Middle + kJoystick_Deadzone,
                                         kJoystick_Maximum, 0, kMotorMaximum_Left));
    analogWrite(kLeftWheel_Backwards, 0);
  } else if (c <= (kJoystick_Middle - kJoystick_Deadzone)) {
    analogWrite(kLeftWheel_Forwards, 0);
    analogWrite(kLeftWheel_Backwards,
                map(kJoystick_Maximum - c, kJoystick_Middle - kJoystick_Deadzone, kJoystick_Maximum,
                    0, kMotorMaximum_Left));
  } else {
    analogWrite(kLeftWheel_Forwards, 0);
    analogWrite(kLeftWheel_Backwards, 0);
  }

  // Right motor
  c = aMsg[6];
  if (c >= (kJoystick_Middle + kJoystick_Deadzone)) {
    analogWrite(kRightWheel_Forwards, map(c, kJoystick_Middle + kJoystick_Deadzone,
                                          kJoystick_Maximum, 0, kMotorMaximum_Right));
    analogWrite(kRightWheel_Backwards, 0);
  } else if (c <= (kJoystick_Middle - kJoystick_Deadzone)) {
    analogWrite(kRightWheel_Forwards, 0);
    analogWrite(kRightWheel_Backwards,
                map(kJoystick_Maximum - c, kJoystick_Middle - kJoystick_Deadzone, kJoystick_Maximum,
                    0, kMotorMaximum_Right));
  } else {
    analogWrite(kRightWheel_Forwards, 0);
    analogWrite(kRightWheel_Backwards, 0);
  }

  memcpy(rxCache, aMsg, kMessageSize_B);
}

static void processByteFromBle(const uint8_t aRxByte) {
  if (numRxBytes == 0 && aRxByte != 0xFF) {
    // Ignore
  } else if (numRxBytes < kMessageSize_B) {
    rxBuffer[numRxBytes] = aRxByte;
    numRxBytes++;
  } else {
    // We've somehow received too many bytes between start bytes. Clear the buffer and try again.
    memset(rxBuffer, 0, sizeof(rxBuffer));
    numRxBytes = 0;
  }

  if (numRxBytes == kMessageSize_B) {
    // We've received enough information to parse out a full message
    firstMsgRx = true;
    parseBleMessage(rxBuffer);

    // Clear the buffer to make room for the next message
    memset(rxBuffer, 0, sizeof(rxBuffer));
    numRxBytes = 0;
  }
}

/* Setup and Loop ------------------------------------------------------------------------------- */

void setup() {
  // Setup serial
  Serial.begin(115200);
  // NOTE: This serial port is shared between both the BLE adapter (e.g., HM-10 module), and the
  // Serial Monitor that is available when you connect the Arduino to a computer. This means if you
  // print anything to the serial, it will go to BOTH the computer and the controller. This should
  // be fine, because the controller doesn't currently parse any input over BLE.

  // Motor setup
  pinMode(kRightWheel_Backwards, OUTPUT);
  pinMode(kRightWheel_Forwards, OUTPUT);
  pinMode(kLeftWheel_Forwards, OUTPUT);
  pinMode(kLeftWheel_Backwards, OUTPUT);

  armServo.attach(kPinArmServo);
  gateServo.attach(kPinGateServo);
}

void loop() {
  if (Serial.available() > 0) {
    processByteFromBle(Serial.read());
  } else if (firstMsgRx) {
    parseBleMessage(rxCache);
    delay(8);
  }
}
