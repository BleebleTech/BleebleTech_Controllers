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

/**
 * @brief Uncomment this macro to enable debugging features. These can help you figure out if
 * anything is going wrong, but may reduce the performance of the program.
 */
// #define DEBUG

/* Types ---------------------------------------------------------------------------------------- */

class Controller {
 protected:
  static constexpr size_t kMessageSize_B = 7;

  uint8_t rxBuffer[kMessageSize_B] = {};
  size_t numRxBytes = 0;

 public:
  boolean btnLeftUp;
  boolean btnLeftRight;
  boolean btnLeftDown;
  boolean btnLeftLeft;
  boolean btnRightUp;
  boolean btnRightRight;
  boolean btnRightDown;
  boolean btnRightLeft;

  boolean btnLeftShoulder;
  boolean btnRightShoulder;

  boolean btnStart;
  boolean btnSelect;

  long joyLeftX = 128;  // 0 to 254 (left to right)
  long joyLeftY = 128;  // 0 to 254 (down to up)
  boolean joyLeftBtn;

  long joyRightX = 128;  // 0 to 254 (left to right)
  long joyRightY = 128;  // 0 to 254 (down to up)
  boolean joyRightBtn;

  void update() {
    // Collect individual bytes over BLE until a full message is received
    if (numRxBytes != kMessageSize_B) {
      while (Serial.available() > 0) {
        uint8_t rxByte = Serial.read();

        if (numRxBytes == 0 && rxByte != 0xFF) {
          // Ignore input until we see the start of a message
        } else if (numRxBytes < kMessageSize_B) {
          // Append the byte to the message buffer
          rxBuffer[numRxBytes++] = rxByte;
        } else {
          // We've somehow received too many bytes between start bytes.
          // Clear the buffer and try again.
          memset(rxBuffer, 0, sizeof(rxBuffer));
          numRxBytes = 0;
        }
      }
      // else {
      //   // Wait for data
      delay(10);
      // }
    } else {
#ifdef DEBUG
      // Print received message to the console
      char s[48] = {0};
      snprintf(s, 47, "RX: [0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x]\n", rxBuffer[0],
               rxBuffer[1], rxBuffer[2], rxBuffer[3], rxBuffer[4], rxBuffer[5], rxBuffer[6]);
      Serial.println(s);
#endif

      // Update the member variables to reflect the current state of the controller
      btnLeftShoulder = (rxBuffer[1] & 0x01);
      btnRightShoulder = (rxBuffer[1] & 0x02);
      btnStart = (rxBuffer[1] & 0x04);
      btnSelect = (rxBuffer[1] & 0x08);
      joyLeftBtn = (rxBuffer[1] & 0x10);
      joyRightBtn = (rxBuffer[1] & 0x20);

      btnLeftUp = (rxBuffer[2] & 0x01);
      btnLeftRight = (rxBuffer[2] & 0x02);
      btnLeftDown = (rxBuffer[2] & 0x04);
      btnLeftLeft = (rxBuffer[2] & 0x08);
      btnRightUp = (rxBuffer[2] & 0x10);
      btnRightRight = (rxBuffer[2] & 0x20);
      btnRightDown = (rxBuffer[2] & 0x40);
      btnRightLeft = (rxBuffer[2] & 0x80);

      joyLeftX = rxBuffer[3];
      joyLeftY = rxBuffer[4];
      joyRightX = rxBuffer[5];
      joyRightY = rxBuffer[6];

      // Clear the message buffer to make room for the next message
      memset(rxBuffer, 0, sizeof(rxBuffer));
      numRxBytes = 0;
    }
  }
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
static constexpr uint8_t kArmServoMax = 110;
static constexpr uint8_t kLeftPaddleServoMin = 0;
static constexpr uint8_t kLeftPaddleServoMax = 90;
static constexpr uint8_t kRightPaddleServoMin = 90;
static constexpr uint8_t kRightPaddleServoMax = 180;

static constexpr uint16_t kArmServoSpeed = 1;
static constexpr uint16_t kPaddleServoSpeed = 6;

/* Variables ------------------------------------------------------------------------------------ */

static Servo armServo;
static Servo leftPaddleServo;
static Servo rightPaddleServo;

static uint8_t armServoPos = 90;
static uint8_t leftPaddleServoPos = 90;
static uint8_t rightPaddleServoPos = 90;

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
  /* ---------------------------------- */
  /*         Update Controller          */
  /* ---------------------------------- */

  controller.update();

  /* ---------------------------------- */
  /*           Control Robot            */
  /* ---------------------------------- */

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
  /* ---------------------------------- */
  /*             TANK DRIVE             */
  /* ---------------------------------- */

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
  /* ---------------------------------- */
  /*        SINGLE JOYSTICK DRIVE       */
  /* ---------------------------------- */

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
}
