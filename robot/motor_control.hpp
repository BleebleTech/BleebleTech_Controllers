/**
 * File: motor_control.hpp
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file contains constants and functions for controlling motors on the robot.
 */

/* Includes ------------------------------------------------------------------------------------- */

#include "controller_handling.hpp"

/* Constants ------------------------------------------------------------------------------------ */

// Motor control pins connected to H-Bridge motor driver
static constexpr int kRightWheel_Backwards = 3;
static constexpr int kRightWheel_Forwards = 5;
static constexpr int kLeftWheel_Forwards = 6;
static constexpr int kLeftWheel_Backwards = 11;

// Motor control configuration
static constexpr uint8_t kMotorMaximum = 254;
// Adjust either of these down (NOT ABOVE 255) if one motor is faster than the other
static constexpr uint8_t kMotorMaximum_Left = 255;
static constexpr uint8_t kMotorMaximum_Right = 255;

static constexpr uint8_t kMotorMaxPercent = 100;

/* Constants ------------------------------------------------------------------------------------ */

static uint8_t motorLimitPercent = kMotorMaxPercent;

/* Functions
   ------------------------------------------------------------------------------------ */

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
    motorLimitPercent = kMotorMaximum;
  }
}

void setLeftMotor(const long aValue) {
  uint8_t motorVal = min(aValue, motorLimitPercent);

  if (motorVal > 0) {
    analogWrite(kLeftWheel_Forwards, map(motorVal, 0, kMotorMaxPercent, 0, kMotorMaximum_Left));
    analogWrite(kLeftWheel_Backwards, 0);
  } else if (motorVal < 0) {
    analogWrite(kLeftWheel_Forwards, 0);
    analogWrite(kLeftWheel_Backwards, map(motorVal, 0, -kMotorMaxPercent, 0, kMotorMaximum_Left));
  } else {
    analogWrite(kLeftWheel_Forwards, 0);
    analogWrite(kLeftWheel_Backwards, 0);
  }
}

void setRightMotor(const long aValue) {
  uint8_t motorVal = min(aValue, motorLimitPercent);

  if (motorVal > 0) {
    analogWrite(kRightWheel_Forwards, map(motorVal, 0, kMotorMaxPercent, 0, kMotorMaximum_Right));
    analogWrite(kRightWheel_Backwards, 0);
  } else if (motorVal < 0) {
    analogWrite(kRightWheel_Forwards, 0);
    analogWrite(kRightWheel_Backwards, map(motorVal, 0, -kMotorMaxPercent, 0, kMotorMaximum_Right));
  } else {
    analogWrite(kRightWheel_Forwards, 0);
    analogWrite(kRightWheel_Backwards, 0);
  }
}
