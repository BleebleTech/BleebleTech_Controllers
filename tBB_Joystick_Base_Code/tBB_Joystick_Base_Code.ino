/**
 * File: tBB_Joystick_Base_Code.ino
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file serves as an example for robots controlled by theBasicBot's ESP32-based controller.
 */

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
  setupDriving();
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
  controlMotors(controller);
  controlAutonomous(controller);
}
