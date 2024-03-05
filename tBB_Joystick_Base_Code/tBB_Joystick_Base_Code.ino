/**
 * File: tBB_Joystick_Base_Code.ino
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file serves as an example for robots controlled by theBasicBot's ESP32-based controller.
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

/**
 * @brief Uncomment this macro to enable debugging features. These can help you figure out if
 * anything is going wrong, but may reduce the performance of the program.
 */
// #define DEBUG

/* Includes ------------------------------------------------------------------------------------- */

#include "1_controller_handling.hpp"
#include "2_motor_control.hpp"
#include "3_driving.hpp"
#include "4_auto_drive.hpp"
#include "5_lights.hpp"
#include "6_servo_control.hpp"
#include "7_sensors.hpp"

/* Variables ------------------------------------------------------------------------------------ */

static Controller controller = {};

/* Setup and Loop ------------------------------------------------------------------------------- */

void setup() {
  // Setup serial
  Serial.begin(9600);
  Serial.println("tBB Joystick Base Code");
  // NOTE: This serial port is shared between both the BLE adapter (e.g., HM-10 module), and the
  // Serial Monitor that is available when you connect the Arduino to a computer. This means if you
  // print anything to the serial, it will go to BOTH the computer and the controller. This should
  // be fine, because the controller doesn't currently parse any input over BLE.

  setupMotors();
  setupServos();
  setupLights();
  setupSensors();
}

void loop() {
  /* ---------------------------------- */
  /*         Update Controller          */
  /* ---------------------------------- */

  controller.update();

  /* ---------------------------------- */
  /*           Control Robot            */
  /* ---------------------------------- */

  controlServo1(controller);
  controlServo2(controller);
  controlLights(controller);

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
