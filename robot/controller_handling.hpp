/**
 * File: controller_handling.hpp
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file contains the class definition of a controller object, and defines the methods to handle
 * data received from the controller over BLE.
 */

#ifndef CONTROLLER_HANDLING_HPP
#define CONTROLLER_HANDLING_HPP

/* Types ---------------------------------------------------------------------------------------- */

class Controller {
 protected:
  static constexpr size_t kMessageSize_B = 7;

  uint8_t rxBuffer[kMessageSize_B] = {};
  size_t numRxBytes = 0;

  static constexpr uint8_t kJoystickMinimum = 0;
  static constexpr uint8_t kJoystickMiddle = 127;
  static constexpr uint8_t kJoystickMaximum = 254;
  static constexpr uint8_t kJoystickDeadzone = 16;

  int8_t parseJoystickValue(const uint8_t aAxis) {
    if (aAxis <= kJoystickMiddle - kJoystickDeadzone) {
      return map(aAxis, kJoystickMinimum, kJoystickMiddle + kJoystickDeadzone, -100, 0);
    } else if (aAxis >= kJoystickMiddle + kJoystickDeadzone) {
      return map(aAxis, kJoystickMiddle + kJoystickDeadzone, kJoystickMaximum, 0, 100);
    } else {
      return 0;
    }
  }

 public:
  boolean btnLeftUp;
  boolean btnLeftRight;
  boolean btnLeftDown;
  boolean btnLeftLeft;
  boolean btnRightUp;
  boolean btnRightRight;
  boolean btnRightDown;
  boolean btnRightLeft;

  boolean btnLeftShoulder;
  boolean btnRightShoulder;

  boolean btnMidLeft;
  boolean btnMidRight;

  int8_t joyLeftX;  // -100 to +100 (left to right)
  int8_t joyLeftY;  // -100 to +100 (down to up)
  boolean joyLeftBtn;

  int8_t joyRightX;  // -100 to +100 (left to right)
  int8_t joyRightY;  // -100 to +100 (down to up)
  boolean joyRightBtn;

  void update() {
    // Collect individual bytes over BLE until a full message is received
    if (numRxBytes != kMessageSize_B) {
      while (Serial.available() > 0) {
        uint8_t rxByte = Serial.read();

        if (numRxBytes == 0 && rxByte != 0xFF) {
          // Ignore input until we see the start of a message
        } else if (numRxBytes < kMessageSize_B) {
          // Append the byte to the message buffer
          rxBuffer[numRxBytes++] = rxByte;
        } else {
          // We've somehow received too many bytes between start bytes.
          // Clear the buffer and try again.
          memset(rxBuffer, 0, sizeof(rxBuffer));
          numRxBytes = 0;
        }
      }
      // else {
      //   // Wait for data
      delay(8);
      // }
    } else {
#ifdef DEBUG
      // Print received message to the console
      char s[48] = {0};
      snprintf(s, 47, "RX: [0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x]\n", rxBuffer[0],
               rxBuffer[1], rxBuffer[2], rxBuffer[3], rxBuffer[4], rxBuffer[5], rxBuffer[6]);
      Serial.println(s);
#endif

      // Update the member variables to reflect the current state of the controller
      btnLeftShoulder = (rxBuffer[1] & 0x01);
      btnRightShoulder = (rxBuffer[1] & 0x02);
      btnMidRight = (rxBuffer[1] & 0x04);
      btnMidLeft = (rxBuffer[1] & 0x08);
      joyLeftBtn = (rxBuffer[1] & 0x10);
      joyRightBtn = (rxBuffer[1] & 0x20);

      btnLeftUp = (rxBuffer[2] & 0x01);
      btnLeftRight = (rxBuffer[2] & 0x02);
      btnLeftDown = (rxBuffer[2] & 0x04);
      btnLeftLeft = (rxBuffer[2] & 0x08);
      btnRightUp = (rxBuffer[2] & 0x10);
      btnRightRight = (rxBuffer[2] & 0x20);
      btnRightDown = (rxBuffer[2] & 0x40);
      btnRightLeft = (rxBuffer[2] & 0x80);

      joyLeftX = parseJoystickValue(rxBuffer[3]);
      joyLeftY = parseJoystickValue(rxBuffer[4]);
      joyRightX = parseJoystickValue(rxBuffer[5]);
      joyRightY = parseJoystickValue(rxBuffer[6]);

      // Clear the message buffer to make room for the next message
      memset(rxBuffer, 0, sizeof(rxBuffer));
      numRxBytes = 0;
    }
  }
};

#endif
