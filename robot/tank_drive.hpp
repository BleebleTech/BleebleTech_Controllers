/**
 * File: tank_drive.hpp
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file contains a function to map a controller's joysticks to a "tank drive" control scheme.
 */

/* Includes ------------------------------------------------------------------------------------- */

#include "controller_handling.hpp"

/* Functions ------------------------------------------------------------------------------------ */

void tankDrive(const Controller& aController) {
  setLeftMotor(aController.joyLeftY);
  setRightMotor(aController.joyLeftY);
}
