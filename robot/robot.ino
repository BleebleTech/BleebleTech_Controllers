/**
 * File: robot.ino
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file serves as an example for robots controlled by theBasicBot's ESP32-based controller.
 */

/* Includes ------------------------------------------------------------------------------------- */

#include "controller_handling.hpp"
#include "motor_control.hpp"
#include "servo_control.hpp"
#include "single_joystick_drive.hpp"
#include "tank_drive.hpp"

/* Macros --------------------------------------------------------------------------------------- */

/**
 * @brief Uncomment this macro to enable "Tank Drive" mode. If left comment out, the robot will
 * remain in the default "Single Joystick Drive" mode.
 *
 * @remark "Tank Drive" mode requires a controller with 2 joysticks, which would mean a tBB Advanced
 * controller, but not a tBB Basic controller.
 */
// #define TANK_DRIVE

/**
 * @brief Uncomment this macro to enable debugging features. These can help you figure out if
 * anything is going wrong, but may reduce the performance of the program.
 */
// #define DEBUG

/* Variables ------------------------------------------------------------------------------------ */

static Controller controller = {};

/* Setup and Loop ------------------------------------------------------------------------------- */

void setup() {
  // Setup serial
  Serial.begin(9600);
  // NOTE: This serial port is shared between both the BLE adapter (e.g., HM-10 module), and the
  // Serial Monitor that is available when you connect the Arduino to a computer. This means if you
  // print anything to the serial, it will go to BOTH the computer and the controller. This should
  // be fine, because the controller doesn't currently parse any input over BLE.

  setupMotors();
  setupServos();
}

void loop() {
  /* ---------------------------------- */
  /*         Update Controller          */
  /* ---------------------------------- */

  controller.update();

  /* ---------------------------------- */
  /*           Control Robot            */
  /* ---------------------------------- */
  controlArmServo(controller);
  controlPaddleServos(controller);

#ifdef TANK_DRIVE
  /* ---------------------------------- */
  /*             TANK DRIVE             */
  /* ---------------------------------- */
  tankDrive(controller);
#else
  /* ---------------------------------- */
  /*        SINGLE JOYSTICK DRIVE       */
  /* ---------------------------------- */
  singleJoystickDrive(controller);
#endif
}
