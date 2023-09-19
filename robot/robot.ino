/**
 * File: robot.ino
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file serves as an example for robots controlled by theBasicBot's ESP32-based controller.
 */

/* Constants ------------------------------------------------------------------------------------ */

// Motor control pins connected to H-Bridge motor driver
static constexpr int kRightWheel_Backwards = 3;
static constexpr int kRightWheel_Forwards = 5;
static constexpr int kLeftWheel_Forwards = 6;
static constexpr int kLeftWheel_Backwards = 11;

// Message decoding configuration
static constexpr int kMessageSize_B = 7;

/* Variables ------------------------------------------------------------------------------------ */

static uint8_t rxBuffer[kMessageSize_B] = {};
static size_t numRxBytes = 0;

/* Functions ------------------------------------------------------------------------------------ */
static void parseBleMessage() {
  if (numRxBytes != kMessageSize_B) {
    // The buffer doesn't contain the right number of bytes! We can't parse the message, since we
    // don't know what is in the buffer.
  } else {
    // LEAVE THIS COMMENTED OUT IF YOU'RE NOT USING IT!
    // Print out the buffer for the user to see
    /*
    char s[48] = {0};
    snprintf(s, 47, "RX: [0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x]\n", rxBuffer[0],
             rxBuffer[1], rxBuffer[2], rxBuffer[3], rxBuffer[4], rxBuffer[5], rxBuffer[6]);
    Serial.println(s);
    */
  }
}

static void processByteFromBle(const uint8_t aRxByte) {
  if (numRxBytes == 0 && aRxByte != 0xFF) {
    // Ignore
  } else if (numRxBytes < kMessageSize_B) {
    rxBuffer[numRxBytes] = aRxByte;
    numRxBytes++;
  } else {
    // We've somehow received too many bytes between start bytes. Clear the buffer and try again.
    memset(rxBuffer, 0, sizeof(rxBuffer));
    numRxBytes = 0;
  }

  if (numRxBytes == kMessageSize_B) {
    // We've received enough information to parse out a full message
    parseBleMessage();

    // Clear the buffer to make room for the next message
    memset(rxBuffer, 0, sizeof(rxBuffer));
    numRxBytes = 0;
  }
}

/* Setup and Loop ------------------------------------------------------------------------------- */

void setup() {
  // Setup serial
  Serial.begin(115200);
  // NOTE: This serial port is shared between both the BLE adapter (e.g., HM-10 module), and the
  // Serial Monitor that is available when you connect the Arduino to a computer. This means if you
  // print anything to the serial, it will go to BOTH the computer and the controller. This should
  // be fine, because the controller doesn't currently parse any input over BLE.

  // Motor setup
  pinMode(kRightWheel_Backwards, OUTPUT);
  pinMode(kRightWheel_Forwards, OUTPUT);
  pinMode(kLeftWheel_Forwards, OUTPUT);
  pinMode(kLeftWheel_Backwards, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    processByteFromBle(Serial.read());
  }
}
