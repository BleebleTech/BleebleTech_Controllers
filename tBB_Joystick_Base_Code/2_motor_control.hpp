#pragma once
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
static constexpr unsigned int kMotorMaximum_Left = 255;
static constexpr unsigned int kMotorMaximum_Right = 255;

// Motor control configuration
static constexpr int kMotorMaxPercent = 100;

/* Variables ------------------------------------------------------------------------------------ */

static double motorLimitFactor = kMotorMaxPercent / 100.0f;

/* Functions ------------------------------------------------------------------------------------ */

void setupMotors() {
  pinMode(kRightWheel_Backwards, OUTPUT);
  pinMode(kRightWheel_Forwards, OUTPUT);
  pinMode(kLeftWheel_Forwards, OUTPUT);
  pinMode(kLeftWheel_Backwards, OUTPUT);
}

void setMotorLimit(const unsigned int aMax) {
  if (aMax <= kMotorMaxPercent) {
    motorLimitFactor = aMax / 100.0f;
  } else {
    motorLimitFactor = kMotorMaxPercent / 100.0f;
  }
}

void setLeftMotor(const int aValue) {
  int motorVal = max(min(aValue, kMotorMaxPercent), -kMotorMaxPercent);

  if (motorVal > 0) {
    analogWrite(kLeftWheel_Forwards,
                map(motorVal, 0, kMotorMaxPercent, 0, kMotorMaximum_Left * motorLimitFactor));
    analogWrite(kLeftWheel_Backwards, 0);
  } else if (motorVal < 0) {
    analogWrite(kLeftWheel_Forwards, 0);
    analogWrite(kLeftWheel_Backwards,
                map(motorVal, 0, -kMotorMaxPercent, 0, kMotorMaximum_Left * motorLimitFactor));
  } else {
    analogWrite(kLeftWheel_Forwards, 0);
    analogWrite(kLeftWheel_Backwards, 0);
  }
}

void setRightMotor(const int aValue) {
  int motorVal = max(min(aValue, kMotorMaxPercent), -kMotorMaxPercent);

  if (motorVal > 0) {
    analogWrite(kRightWheel_Forwards,
                map(motorVal, 0, kMotorMaxPercent, 0, kMotorMaximum_Right * motorLimitFactor));
    analogWrite(kRightWheel_Backwards, 0);
  } else if (motorVal < 0) {
    analogWrite(kRightWheel_Forwards, 0);
    analogWrite(kRightWheel_Backwards,
                map(motorVal, 0, -kMotorMaxPercent, 0, kMotorMaximum_Right * motorLimitFactor));
  } else {
    analogWrite(kRightWheel_Forwards, 0);
    analogWrite(kRightWheel_Backwards, 0);
  }
}
