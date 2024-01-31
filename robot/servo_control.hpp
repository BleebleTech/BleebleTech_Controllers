/**
 * File: servo_control.hpp
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file contains functions to control both the arm and paddle servos of the robot.
 */

// TODO: Abstraction of speed (make 0-100, like motors)
// TODO: Make generic, leave layout/template

/* Includes ------------------------------------------------------------------------------------- */

#include <Servo.h>

#include "controller_handling.hpp"

/* Constants ------------------------------------------------------------------------------------ */

// Servo configuration
static constexpr int kPinArmServo = 10;
static constexpr int kPinLeftPaddleServo = 8;
static constexpr int kPinRightPaddleServo = 9;

static constexpr uint8_t kArmServoMin = 60;
static constexpr uint8_t kArmServoMax = 115;
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

/* Functions ------------------------------------------------------------------------------------ */

void setupServos() {
  armServo.attach(kPinArmServo);
  leftPaddleServo.attach(kPinLeftPaddleServo);
  rightPaddleServo.attach(kPinRightPaddleServo);
}

void controlArmServo(const Controller& aController) {
  // Servos
  if (aController.btnRightRight && armServoPos < kArmServoMax) {
    armServoPos += kArmServoSpeed;
  } else if (aController.btnRightDown && armServoPos > kArmServoMin) {
    armServoPos -= kArmServoSpeed;
  }

  armServo.write(armServoPos);
}

void controlPaddleServos(const Controller& aController) {
  if (aController.btnRightUp && leftPaddleServoPos < kLeftPaddleServoMax) {
    leftPaddleServoPos += kPaddleServoSpeed;
    rightPaddleServoPos = kRightPaddleServoMax - leftPaddleServoPos;
  } else if (aController.btnRightLeft && leftPaddleServoPos > kLeftPaddleServoMin) {
    leftPaddleServoPos -= kPaddleServoSpeed;
    rightPaddleServoPos = kRightPaddleServoMax - leftPaddleServoPos;
  }

  leftPaddleServo.write(leftPaddleServoPos);
  rightPaddleServo.write(rightPaddleServoPos);
}
