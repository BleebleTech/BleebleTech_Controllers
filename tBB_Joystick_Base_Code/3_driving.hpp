#pragma once
/**
 * File: 3_driving.hpp
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file contains functions to map a controller's joystick(s) for driving the Basic Bot.
 * The tank drive mode requires 2 joysticks, so it can only be used with the Advanced Controller.
 */

/* Config --------------------------------------------------------------------------------------- */

/**
 * @brief Uncomment this macro to enable "Tank Drive" mode. If left comment out, the robot will
 * remain in the default "Single Joystick Drive" mode.
 *
 * @remark "Tank Drive" mode requires a controller with 2 joysticks, which would mean a tBB Advanced
 * controller, but not a tBB Basic controller.
 */
// #define TANK_DRIVE

/* Includes ------------------------------------------------------------------------------------- */

#include "1_controller_handling.hpp"
#include "2_motor_control.hpp"

/* Constants ------------------------------------------------------------------------------------ */

static constexpr int kFastModeMotorPercent = 100;
static constexpr int kSlowModeMotorPercent = 50;

static constexpr unsigned long kSpeedToggleDebounceDelayMs = 100;

/* Variables ------------------------------------------------------------------------------------ */

static boolean isFastModeEnabled = true;
static boolean wasSpeedToggleBtnPressed = false;
static unsigned long lastSpeedToggleButtonTime = 0;

/* Functions ------------------------------------------------------------------------------------ */

void singleJoystickDrive(const Controller& aController) {
  if (aController.joyLeftY >= 0) {
    if (aController.joyLeftX >= 0) {
      setLeftMotor(min(aController.joyLeftY + aController.joyLeftX, 100));
      setRightMotor(aController.joyLeftY - aController.joyLeftX);
    } else if (aController.joyLeftX < 0) {
      setLeftMotor(aController.joyLeftY + aController.joyLeftX);
      setRightMotor(min(aController.joyLeftY - aController.joyLeftX, 100));
    }
  } else if (aController.joyLeftY < 0) {
    if (aController.joyLeftX >= 0) {
      setLeftMotor(aController.joyLeftY + aController.joyLeftX);
      setRightMotor(max(aController.joyLeftY - aController.joyLeftX, -100));
    } else if (aController.joyLeftX < 0) {
      setLeftMotor(max(aController.joyLeftY + aController.joyLeftX, -100));
      setRightMotor(aController.joyLeftY - aController.joyLeftX);
    }
  }
}

void tankDrive(const Controller& aController) {
  setLeftMotor(aController.joyLeftY);
  setRightMotor(aController.joyLeftY);
}

void setFastMode() {
  setMotorLimit(kFastModeMotorPercent);
  isFastModeEnabled = true;
}

void setSlowMode() {
  setMotorLimit(kSlowModeMotorPercent);
  isFastModeEnabled = false;
}

void toggleMotorSpeed() {
  if (isFastModeEnabled) {
    setSlowMode();
  } else {
    setFastMode();
  }
}

void setupDriving() { setFastMode(); }

void controlMotors(const Controller& aController) {
  unsigned long currentTime = millis();
  if (aController.joyLeftBtn && !wasSpeedToggleBtnPressed &&
      currentTime - lastSpeedToggleButtonTime > kSpeedToggleDebounceDelayMs) {
    lastSpeedToggleButtonTime = millis();
    wasSpeedToggleBtnPressed = true;
    toggleMotorSpeed();
  } else if (!aController.joyLeftBtn) {
    wasSpeedToggleBtnPressed = false;
  }

#ifdef TANK_DRIVE
  /* ---------------------------------- */
  /*             TANK DRIVE             */
  /* ---------------------------------- */
  tankDrive(aController);
#else
  /* ---------------------------------- */
  /*        SINGLE JOYSTICK DRIVE       */
  /* ---------------------------------- */
  singleJoystickDrive(aController);
#endif
}

void driveForward() {
  setLeftMotor(100);
  setRightMotor(100);
}

void driveBackwards() {
  setLeftMotor(-100);
  setRightMotor(-100);
}

void stopDriving() {
  setLeftMotor(0);
  setRightMotor(0);
}

void turnRight() {
  setLeftMotor(-100);
  setRightMotor(100);
}

void turnLeft() {
  setLeftMotor(100);
  setRightMotor(-100);
}
