/**
 * File: 2_motor_control.hpp
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file contains constants and functions for controlling motors on the robot.
 */

/* Includes ------------------------------------------------------------------------------------- */

#include "1_controller_handling.hpp"

/* Constants ------------------------------------------------------------------------------------ */

// Motor control pins connected to H-Bridge motor driver
static constexpr int kLeftWheel_Backwards = 3;
static constexpr int kLeftWheel_Forwards = 5;
static constexpr int kRightWheel_Forwards = 6;
static constexpr int kRightWheel_Backwards = 11;

// Adjust either of these down (NOT ABOVE 255) if one motor is faster than the other
static constexpr uint8_t kMotorMaximum_Left = 255;
static constexpr uint8_t kMotorMaximum_Right = 255;

// Motor control configuration
static constexpr uint8_t kMotorMaxPercent = 100;

/* Variables ------------------------------------------------------------------------------------ */

static uint8_t motorLimitPercent = kMotorMaxPercent;

/* Functions ------------------------------------------------------------------------------------ */

void setupMotors() {
  pinMode(kRightWheel_Backwards, OUTPUT);
  pinMode(kRightWheel_Forwards, OUTPUT);
  pinMode(kLeftWheel_Forwards, OUTPUT);
  pinMode(kLeftWheel_Backwards, OUTPUT);
}

void setMotorLimit(const uint8_t aMax) {
  if (aMax <= kMotorMaxPercent) {
    motorLimitPercent = aMax;
  } else {
    motorLimitPercent = kMotorMaxPercent;
  }
}

void setLeftMotor(const uint8_t aValue) {
  uint8_t motorVal = max(min(aValue, motorLimitPercent), -motorLimitPercent);

  if (motorVal > 0) {
    analogWrite(kLeftWheel_Forwards, map(motorVal, 0, 100, 0, kMotorMaximum_Left));
    analogWrite(kLeftWheel_Backwards, 0);
  } else if (motorVal < 0) {
    analogWrite(kLeftWheel_Forwards, 0);
    analogWrite(kLeftWheel_Backwards, map(motorVal, 0, -100, 0, kMotorMaximum_Left));
  } else {
    analogWrite(kLeftWheel_Forwards, 0);
    analogWrite(kLeftWheel_Backwards, 0);
  }
}

void setRightMotor(const uint8_t aValue) {
  uint8_t motorVal = max(min(aValue, motorLimitPercent), -motorLimitPercent);

  if (motorVal > 0) {
    analogWrite(kRightWheel_Forwards, map(motorVal, 0, 100, 0, kMotorMaximum_Right));
    analogWrite(kRightWheel_Backwards, 0);
  } else if (motorVal < 0) {
    analogWrite(kRightWheel_Forwards, 0);
    analogWrite(kRightWheel_Backwards, map(motorVal, 0, -100, 0, kMotorMaximum_Right));
  } else {
    analogWrite(kRightWheel_Forwards, 0);
    analogWrite(kRightWheel_Backwards, 0);
  }
}
