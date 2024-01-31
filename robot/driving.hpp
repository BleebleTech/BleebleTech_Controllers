/**
 * File: driving.hpp
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file contains a function to map a controller's joystick(s) to drive control scheme.
 */

/* Includes ------------------------------------------------------------------------------------- */

#include "controller_handling.hpp"

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
