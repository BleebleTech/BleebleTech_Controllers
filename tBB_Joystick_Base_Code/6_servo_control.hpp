#pragma once
/**
 * File: 6_servo_control.hpp
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file contains functions to control 2 servo motors.
 */

/* Includes ------------------------------------------------------------------------------------- */

#include <Adafruit_PWMServoDriver.h>

#include "1_controller_handling.hpp"

/* Constants and Variables ---------------------------------------------------------------------- */


/* ---------------------------------- */
/*           Driver Config            */
/* ---------------------------------- */

static Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

static constexpr uint32_t kServoDriverOscillatorFrequencyHz = 27000000;
static constexpr float kServoDriverPwmFrequencyHz = 50.0f;
static constexpr uint16_t kServoPulseMin = 100;
static constexpr uint16_t kServoPulseMax = 500;
static constexpr uint16_t kServoAngleMin = 0;
static constexpr uint16_t kServoAngleMax = 180;

/* ---------------------------------- */
/*           Servo 1 Config           */
/* ---------------------------------- */

static constexpr int kServo1Channel = 0;

static uint8_t servo1Pos = 90;
static constexpr uint8_t kServo1MinDeg = 1;    // Minimum 1
static constexpr uint8_t kServo1MaxDeg = 180;  // Maximum 180
static constexpr int16_t kServo1Speed = 5;

/* ---------------------------------- */
/*           Servo 2 Config           */
/* ---------------------------------- */

static constexpr int kServo2Channel = 1;

static uint8_t servo2Pos = 90;
static constexpr uint8_t kServo2MinDeg = 45;   // Minimum 0
static constexpr uint8_t kServo2MaxDeg = 130;  // Maximum 180
static constexpr int16_t kServo2Speed = 2;

/* Functions ------------------------------------------------------------------------------------ */

void setupServos() {
  servoDriver.begin();
  servoDriver.setOscillatorFrequency(kServoDriverOscillatorFrequencyHz);
  servoDriver.setPWMFreq(kServoDriverPwmFrequencyHz);

  Serial.print("Servo1 starting position = ");
  Serial.print(servo1Pos);
  Serial.println(" deg");

  Serial.print("Servo2 starting position = ");
  Serial.print(servo2Pos);
  Serial.println(" deg");
}

void setServoAngle(const uint8_t aChannel, const uint16_t aAngle) {
  const uint16_t limitedAngle = min(aAngle, 180);
  servoDriver.setPWM(aChannel, 0, map(limitedAngle, kServoAngleMin, kServoAngleMax, kServoPulseMin, kServoPulseMax));
}

void controlServo1(const Controller& aController) {
  if (aController.btnRightRight && servo1Pos < kServo1MaxDeg) {
    servo1Pos += min(kServo1Speed, kServo1MaxDeg - servo1Pos);
    Serial.print("Servo1 = ");
    Serial.print(servo1Pos);
    Serial.println(" deg");
  } else if (aController.btnRightDown && servo1Pos > kServo1MinDeg) {
    servo1Pos -= min(kServo1Speed, servo1Pos - kServo1MinDeg);
    Serial.print("Servo1 = ");
    Serial.print(servo1Pos);
    Serial.println(" deg");
  }
  setServoAngle(kServo1Channel, servo1Pos);
}

void controlServo2(const Controller& aController) {
  if (aController.btnRightUp && servo2Pos < kServo2MaxDeg) {
    servo2Pos += min(kServo2Speed, kServo2MaxDeg - servo2Pos);
    Serial.print("Servo2 = ");
    Serial.print(servo2Pos);
    Serial.println(" deg");
  } else if (aController.btnRightLeft && servo2Pos > kServo2MinDeg) {
    servo2Pos -= min(kServo2Speed, servo2Pos - kServo2MinDeg);
    Serial.print("Servo2 = ");
    Serial.print(servo2Pos);
    Serial.println(" deg");
  }
  setServoAngle(kServo2Channel, servo2Pos);
}
