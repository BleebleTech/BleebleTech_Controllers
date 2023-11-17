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

/* Macros --------------------------------------------------------------------------------------- */

/**
 * @brief Uncomment this macro to enable "Tank Drive" mode. If left comment out, the robot will
 * remain in the default "Single Joystick Drive" mode.
 *
 * @remark "Tank Drive" mode requires a controller with 2 joysticks, which would mean a tBB Advanced
 * controller, but not a tBB Basic controller.
 */
// #define TANK_DRIVE

/* Types ---------------------------------------------------------------------------------------- */

struct Controller {
  /* ---- Basic and Advanced Cntroller ---- */
  boolean btnRightUp;
  boolean btnRightRight;
  boolean btnRightDown;
  boolean btnRightLeft;

  boolean btnStart;
  boolean btnSelect;

  long joyLeftX;  // -1000 to +1000 (left to right)
  long joyLeftY;  // -1000 to +1000 (down to up)
  boolean joyLeftBtn;

  /* ---- Advanced Controller ONLY ---- */
  boolean btnLeftUp;
  boolean btnLeftRight;
  boolean btnLeftDown;
  boolean btnLeftLeft;

  boolean btnLeftShoulder;
  boolean btnRightShoulder;

  long joyRightX;  // -1000 to +1000 (left to right)
  long joyRightY;  // -1000 to +1000 (down to up)
  boolean joyRightBtn;
};

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
static constexpr int kPinArmServo = 10;
static constexpr int kPinLeftPaddleServo = 8;
static constexpr int kPinRightPaddleServo = 9;

static constexpr uint8_t kArmServoMin = 60;
static constexpr uint8_t kArmServoMax = 120;
static constexpr uint8_t kLeftPaddleServoMin = 0;
static constexpr uint8_t kLeftPaddleServoMax = 90;
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
static Servo leftPaddleServo;
static Servo rightPaddleServo;

static uint8_t armServoPos = 90;
static uint8_t leftPaddleServoPos = 90;
static uint8_t rightPaddleServoPos = 90;

static bool firstMsgRx = false;

static Controller controller = {};

/* Functions ------------------------------------------------------------------------------------ */
static void setLeftMotor(const long aValue) {
  if (aValue > 0) {
    analogWrite(kLeftWheel_Forwards, map(aValue, 0, 1000, 0, kMotorMaximum_Left));
    analogWrite(kLeftWheel_Backwards, 0);
  } else if (aValue < 0) {
    analogWrite(kLeftWheel_Forwards, 0);
    analogWrite(kLeftWheel_Backwards, map(aValue, 0, -1000, 0, kMotorMaximum_Left));
  } else {
    analogWrite(kLeftWheel_Forwards, 0);
    analogWrite(kLeftWheel_Backwards, 0);
  }
}

static void setRightMotor(const long aValue) {
  if (aValue > 0) {
    analogWrite(kRightWheel_Forwards, map(aValue, 0, 1000, 0, kMotorMaximum_Right));
    analogWrite(kRightWheel_Backwards, 0);
  } else if (aValue < 0) {
    analogWrite(kRightWheel_Forwards, 0);
    analogWrite(kRightWheel_Backwards, map(aValue, 0, -1000, 0, kMotorMaximum_Right));
  } else {
    analogWrite(kRightWheel_Forwards, 0);
    analogWrite(kRightWheel_Backwards, 0);
  }
}

static void parseBleMessage(const uint8_t* const aMsg) {
  // LEAVE THIS COMMENTED OUT IF YOU'RE NOT USING IT!
  // Print out the buffer for the user to see
  /*
  char s[48] = {0};
  snprintf(s, 47, "RX: [0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x]\n", aMsg[0],
           aMsg[1], aMsg[2], aMsg[3], aMsg[4], aMsg[5], aMsg[6]);
  Serial.println(s);
  */

  controller.btnLeftShoulder = (aMsg[1] & 0x01);
  controller.btnRightShoulder = (aMsg[1] & 0x02);
  controller.btnStart = (aMsg[1] & 0x04);
  controller.btnSelect = (aMsg[1] & 0x08);
  controller.joyLeftBtn = (aMsg[1] & 0x10);
  controller.joyRightBtn = (aMsg[1] & 0x20);

  controller.btnLeftUp = (aMsg[2] & 0x01);
  controller.btnLeftRight = (aMsg[2] & 0x02);
  controller.btnLeftDown = (aMsg[2] & 0x04);
  controller.btnLeftLeft = (aMsg[2] & 0x08);
  controller.btnRightUp = (aMsg[2] & 0x10);
  controller.btnRightRight = (aMsg[2] & 0x20);
  controller.btnRightDown = (aMsg[2] & 0x40);
  controller.btnRightLeft = (aMsg[2] & 0x80);

  controller.joyLeftX = aMsg[3];
  controller.joyLeftY = aMsg[4];
  controller.joyRightX = aMsg[5];
  controller.joyRightY = aMsg[6];

  // Servos
  if (controller.btnRightRight && armServoPos < kArmServoMax) {
    armServoPos += kArmServoSpeed;
  } else if (controller.btnRightDown && armServoPos > kArmServoMin) {
    armServoPos -= kArmServoSpeed;
  }

  armServo.write(armServoPos);

  if (controller.btnRightUp && leftPaddleServoPos < kLeftPaddleServoMax) {
    leftPaddleServoPos += kPaddleServoSpeed;
    rightPaddleServoPos = kRightPaddleServoMax - leftPaddleServoPos;
  } else if (controller.btnRightLeft && leftPaddleServoPos > kLeftPaddleServoMin) {
    leftPaddleServoPos -= kPaddleServoSpeed;
    rightPaddleServoPos = kRightPaddleServoMax - leftPaddleServoPos;
  }

  leftPaddleServo.write(leftPaddleServoPos);
  rightPaddleServo.write(rightPaddleServoPos);

#ifdef TANK_DRIVE
  /* -------------------------------------- */
  /*               TANK DRIVE               */
  /* -------------------------------------- */

  // Left motor
  if (controller.joyLeftY >= (kJoystick_Middle + kJoystick_Deadzone)) {
    setLeftMotor(map(controller.joyLeftY, kJoystick_Middle + kJoystick_Deadzone, kJoystick_Maximum,
                     0, 1000));
  } else if (controller.joyLeftY <= (kJoystick_Middle - kJoystick_Deadzone)) {
    setLeftMotor(map(kJoystick_Maximum - controller.joyLeftY, kJoystick_Middle - kJoystick_Deadzone,
                     kJoystick_Maximum, 0, -1000));
  } else {
    setLeftMotor(0);
  }

  // Right motor
  if (controller.joyRightY >= (kJoystick_Middle + kJoystick_Deadzone)) {
    setRightMotor(map(controller.joyRightY, kJoystick_Middle + kJoystick_Deadzone,
                      kJoystick_Maximum, 0, 1000));
  } else if (controller.joyRightY <= (kJoystick_Middle - kJoystick_Deadzone)) {
    setRightMotor(map(kJoystick_Maximum - controller.joyRightY,
                      kJoystick_Middle - kJoystick_Deadzone, kJoystick_Maximum, 0, -1000));
  } else {
    setRightMotor(0);
  }
#else
  /* -------------------------------------- */
  /*          SINGLE JOYSTICK DRIVE         */
  /* -------------------------------------- */

  long fb = 0;  // Joystick "Forwards/Backward" value. Positive is forwards, negative is backwards.
  long lr = 0;  // Joystick "Left/Right" value. Positive is right, negative is left.

  // LR
  if (controller.joyLeftX >= (kJoystick_Middle + kJoystick_Deadzone)) {
    lr = map(controller.joyLeftX, kJoystick_Middle + kJoystick_Deadzone, kJoystick_Maximum, 0,
             -1000);
  } else if (controller.joyLeftX <= (kJoystick_Middle - kJoystick_Deadzone)) {
    lr = map(kJoystick_Maximum - controller.joyLeftX, kJoystick_Middle - kJoystick_Deadzone,
             kJoystick_Maximum, 0, 1000);
  } else {
    lr = 0;
  }

  // FB
  if (controller.joyLeftY >= (kJoystick_Middle + kJoystick_Deadzone)) {
    fb =
        map(controller.joyLeftY, kJoystick_Middle + kJoystick_Deadzone, kJoystick_Maximum, 0, 1000);
  } else if (controller.joyLeftY <= (kJoystick_Middle - kJoystick_Deadzone)) {
    fb = map(kJoystick_Maximum - controller.joyLeftY, kJoystick_Middle - kJoystick_Deadzone,
             kJoystick_Maximum, 0, -1000);
  } else {
    fb = 0;
  }

  // Mixing
  if (fb >= 0) {
    if (lr >= 0) {
      setLeftMotor(min(fb + lr, 1000));
      setRightMotor(fb - lr);
    } else if (lr < 0) {
      setLeftMotor(fb + lr);
      setRightMotor(min(fb - lr, 1000));
    }
  } else if (fb < 0) {
    if (lr >= 0) {
      setLeftMotor(fb + lr);
      setRightMotor(max(fb - lr, -1000));
    } else if (lr < 0) {
      setLeftMotor(max(fb + lr, -1000));
      setRightMotor(fb - lr);
    }
  }
#endif

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
  leftPaddleServo.attach(kPinLeftPaddleServo);
  rightPaddleServo.attach(kPinRightPaddleServo);
}

void loop() {
  if (Serial.available() > 0) {
    processByteFromBle(Serial.read());
  } else if (firstMsgRx) {
    parseBleMessage(rxCache);
    delay(8);
  }
}
