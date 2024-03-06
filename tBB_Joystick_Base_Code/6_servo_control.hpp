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

#include <Servo.h>

#include "1_controller_handling.hpp"

/* Constants ------------------------------------------------------------------------------------ */

/* ---------------------------------- */
/*           Servo 1 Config           */
/* ---------------------------------- */

static Servo servo1;
static constexpr int kPinServo1 = 9;

static uint8_t servo1Pos = 90;
static constexpr uint8_t kServo1MinDeg = 1;    // Minimum 1
static constexpr uint8_t kServo1MaxDeg = 180;  // Maximum 180
static constexpr uint16_t kServo1Speed = 1;

/* ---------------------------------- */
/*           Servo 2 Config           */
/* ---------------------------------- */

static Servo servo2;
static constexpr int kPinServo2 = 10;

static uint8_t servo2Pos = 90;
static constexpr uint8_t kServo2MinDeg = 40;   // Minimum 0
static constexpr uint8_t kServo2MaxDeg = 140;  // Maximum 180
static constexpr uint16_t kServo2Speed = 20;

/* Functions ------------------------------------------------------------------------------------ */

void setupServos() {
  servo1.attach(kPinServo1);
  Serial.print("Servo1 starting position = ");
  Serial.print(servo1Pos);
  Serial.println(" deg");

  servo2.attach(kPinServo2);
  Serial.print("Servo2 starting position = ");
  Serial.print(servo2Pos);
  Serial.println(" deg");
}

void controlServo1(const Controller& aController) {
  if (aController.btnRightRight && servo1Pos < kServo1MaxDeg) {
    servo1Pos += min(kServo1Speed, kServo1MaxDeg - servo1Pos);
    Serial.print("Servo1 = ");
    Serial.print(servo1Pos);
    Serial.println(" deg");
  } else if (aController.btnRightDown && servo1Pos > kServo1MinDeg) {
    servo1Pos -= max(kServo1Speed, servo1Pos - kServo1MinDeg);
    Serial.print("Servo1 = ");
    Serial.print(servo1Pos);
    Serial.println(" deg");
  }
  servo1.write(servo1Pos);
}

void controlServo2(const Controller& aController) {
  if (aController.btnRightRight && servo2Pos < kServo2MaxDeg) {
    servo2Pos += min(kServo2Speed, kServo2MaxDeg - servo2Pos);
    Serial.print("Servo2 = ");
    Serial.print(servo2Pos);
    Serial.println(" deg");
  } else if (aController.btnRightDown && servo2Pos > kServo2MinDeg) {
    servo2Pos -= max(kServo2Speed, servo2Pos - kServo2MinDeg);
    Serial.print("Servo2 = ");
    Serial.print(servo2Pos);
    Serial.println(" deg");
  }
  servo2.write(servo2Pos);
}
