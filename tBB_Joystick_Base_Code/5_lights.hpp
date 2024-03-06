#pragma once
/**
 * File: 5_lights.hpp
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file contains functions to control both the arm and paddle servos of the robot.
 */

/* Includes ------------------------------------------------------------------------------------- */

#include "1_controller_handling.hpp"

/* Constants ------------------------------------------------------------------------------------ */

// Lights configuration
static constexpr int kPinLightLeft = 4;
static constexpr int kPinLightRight = 13;

static constexpr unsigned long kToggleDebounceDelayMs = 100;

/* Variables ------------------------------------------------------------------------------------ */

static boolean areLightsOn = false;
static boolean wasToggleBtnPressed = false;
static unsigned long lastToggleButtonTime = 0;

/* Functions ------------------------------------------------------------------------------------ */

void lightsOn() {
  digitalWrite(kPinLightLeft, HIGH);
  digitalWrite(kPinLightRight, HIGH);
  areLightsOn = true;
  Serial.println("Lights On");
}

void lightsOff() {
  digitalWrite(kPinLightLeft, LOW);
  digitalWrite(kPinLightRight, LOW);
  areLightsOn = false;
  Serial.println("Lights Off");
}

void lightsToggle() {
  if (areLightsOn) {
    lightsOff();
  } else {
    lightsOn();
  }
}

void setupLights() {
  pinMode(kPinLightLeft, OUTPUT);
  pinMode(kPinLightRight, OUTPUT);
  lightsOff();
}

void controlLights(const Controller& aController) {
  unsigned long currentTime = millis();
  if (aController.btnMidRight && !wasToggleBtnPressed &&
      currentTime - lastToggleButtonTime > kToggleDebounceDelayMs) {
    lastToggleButtonTime = millis();
    wasToggleBtnPressed = true;
    lightsToggle();
  } else if (!aController.btnMidRight) {
    wasToggleBtnPressed = false;
  }
}
