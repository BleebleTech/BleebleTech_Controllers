/**
 * File: lights.hpp
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file contains functions to control both the arm and paddle servos of the robot.
 */

/* Includes ------------------------------------------------------------------------------------- */

#include "controller_handling.hpp"

/* Constants ------------------------------------------------------------------------------------ */

// Servo configuration
static constexpr int kPinLightLeft = 4;
static constexpr int kPinLightRight = 13;

/* Functions ------------------------------------------------------------------------------------ */

void lightsOn() {
  digitalWrite(kPinLightLeft, HIGH);
  digitalWrite(kPinLightRight, HIGH);
}

void lightsOff() {
  digitalWrite(kPinLightLeft, LOW);
  digitalWrite(kPinLightRight, LOW);
}

void setupLights() {
  pinMode(kPinLightLeft, OUTPUT);
  pinMode(kPinLightRight, OUTPUT);
  lightsOff();
}

void controlLights(const Controller& aController) {
  if (aController.btnMidRight) {
    lightsOn();
  } else if (aController.btnMidLeft) {
    lightsOff();
  }
}
