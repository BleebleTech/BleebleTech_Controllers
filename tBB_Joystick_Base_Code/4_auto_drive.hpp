#pragma once
/**
 * File: 4_auto_drive.hpp
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file contains code needed to create an autonomous driving function based on time delays.
 */

/* Includes ------------------------------------------------------------------------------------- */

#include "1_controller_handling.hpp"
#include "3_driving.hpp"

/* Constants ------------------------------------------------------------------------------------ */

/* Variables ------------------------------------------------------------------------------------ */

static boolean isRunningAuto = false;

/* Functions ------------------------------------------------------------------------------------ */

void runAutonomous() {
  setMotorLimit(50);
  driveForward();
  delay(3000);
  turnRight();
  delay(1000);
  setMotorLimit(80);
  driveBackwards();
  delay(3000);
  turnLeft();
  delay(1000);
  stopDriving();
  delay(5000);

  setFastMode();
}

void controlAutonomous(const Controller& aController) {
  unsigned long currentTime = millis();
  if (aController.btnMidLeft && !isRunningAuto) {
    isRunningAuto = true;
    runAutonomous();
  } else if (!aController.btnMidLeft && isRunningAuto) {
    isRunningAuto = false;
  }
}
