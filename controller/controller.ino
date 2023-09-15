/**
 * File: controller.ino
 * Author: Matthew Allwright, theBasicBot
 * Copyright: 2023
 *
 * Description:
 * This file serves as the base sketch for theBasicBot's ESP32-based controller.
 */

/* Includes ------------------------------------------------------------------------------------- */

#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEUtils.h>

/* Constants ------------------------------------------------------------------------------------ */

// Left button cluster
static constexpr int kPinButtonLeft_Up = 21;
static constexpr int kPinButtonLeft_Right = 19;
static constexpr int kPinButtonLeft_Down = 18;
static constexpr int kPinButtonLeft_Left = 5;

// Right button cluster
static constexpr int kPinButtonRight_Up = 17;
static constexpr int kPinButtonRight_Right = 16;
static constexpr int kPinButtonRight_Down = 4;
static constexpr int kPinButtonRight_Left = 0;

// Shoulder buttons
static constexpr int kPinButtonShoulder_Left = 25;
static constexpr int kPinButtonShoulder_Right = 13;

// Start and select
static constexpr int kPinButtonStart = 15;
static constexpr int kPinButtonSelect = 2;

// Left Joystick
static constexpr int kPinJoystickLeft_X = 26;
static constexpr int kPinJoystickLeft_Y = 27;
static constexpr int kPinJoystickLeft_Button = 32;

// Right Joystick
static constexpr int kPinJoystickRight_X = 14;
static constexpr int kPinJoystickRight_Y = 12;
static constexpr int kPinJoystickRight_Button = 33;

// Arrays for reading inputs in loops
static constexpr std::array<int, 14> kPins_Buttons = {
    kPinButtonLeft_Up,       kPinButtonLeft_Right,     kPinButtonLeft_Down,  kPinButtonLeft_Left,
    kPinButtonRight_Up,      kPinButtonRight_Right,    kPinButtonRight_Down, kPinButtonRight_Left,
    kPinButtonShoulder_Left, kPinButtonShoulder_Right, kPinButtonStart,      kPinButtonSelect,
    kPinJoystickLeft_Button, kPinJoystickRight_Button};
static constexpr std::array<int, 4> kPins_Joysticks = {kPinJoystickLeft_X, kPinJoystickLeft_Y,
                                                       kPinJoystickRight_X, kPinJoystickRight_Y};
// Current encoding can only handle 16 digital values (1b each)
static_assert(kPins_Buttons.size() <= 16);
// Current encoding can only handle 4 analog values (1B each)
static_assert(kPins_Joysticks.size() == 4);

// Status LED
static constexpr int kPinLed_Red = 23;
static constexpr int kPinLed_Green = 22;

// BLE adapter programming port
static constexpr int kPinEspTx_to_BleRx = 34;
static constexpr int kPinEspRx_to_BleTx = 35;

// BLE configuration
static BLEUUID kBleServiceUUID("0000FFE0-0000-1000-8000-00805F9B34FB");
static BLEUUID kBleCharUUID("0000FFE1-0000-1000-8000-00805F9B34FB");
static constexpr int kScanTime_s = 5;
static constexpr int kScanDelay_ms = 2000;

// Generic configuration
static constexpr int kLoopDelay_ms = 64;

/* Variables ------------------------------------------------------------------------------------ */

static bool isBleDeviceConnected = false;
static BLEClient* bleClient;

/* Classes -------------------------------------------------------------------------------------- */

/**
 * @brief
 * Overloads the onConnect(...) and onDisconnect(...) functions, which are called whenever the ESP32
 * connects to or disconnects from another BLE device.
 */
class ControllerBleClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* aClient) { isBleDeviceConnected = true; };
  void onDisconnect(BLEClient* aClient) { isBleDeviceConnected = false; }
};

/**
 * @brief
 * Overloads the onResult(...) method, which is called during a BLE search, whenever the ESP32 hears
 * a new device.
 */
class ControllerBleAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
  }
};

/* Setup and Loop ------------------------------------------------------------------------------- */

void setup() {
  /* Set pin modes -------------------- */

  // Left button cluster
  pinMode(kPinButtonLeft_Up, INPUT);
  pinMode(kPinButtonLeft_Right, INPUT);
  pinMode(kPinButtonLeft_Down, INPUT);
  pinMode(kPinButtonLeft_Left, INPUT);

  // Right button cluster
  pinMode(kPinButtonRight_Up, INPUT);
  pinMode(kPinButtonRight_Right, INPUT);
  pinMode(kPinButtonRight_Down, INPUT);
  pinMode(kPinButtonRight_Left, INPUT);

  // Shoulder buttons
  pinMode(kPinButtonShoulder_Left, INPUT);
  pinMode(kPinButtonShoulder_Right, INPUT);

  // Start and select
  pinMode(kPinButtonStart, INPUT);
  pinMode(kPinButtonSelect, INPUT);

  // Left Joystick
  pinMode(kPinJoystickLeft_X, INPUT);
  pinMode(kPinJoystickLeft_Y, INPUT);
  pinMode(kPinJoystickLeft_Button, INPUT);

  // Right Joystick
  pinMode(kPinJoystickRight_X, INPUT);
  pinMode(kPinJoystickRight_Y, INPUT);
  pinMode(kPinJoystickRight_Button, INPUT);

  // Status LED
  pinMode(kPinLed_Red, OUTPUT);
  pinMode(kPinLed_Green, OUTPUT);

  /* Begin debug serial --------------- */

  Serial.begin(115200);

  /* Scan for BLE devices ------------- */

  Serial.println("Scanning for BLE devices...");

  // Setup BLE objects
  BLEDevice::init("");
  BLEScan* bleScan = BLEDevice::getScan();
  bleClient = BLEDevice::createClient();

  // Configure BLE client settings
  bleClient->setClientCallbacks(new ControllerBleClientCallbacks());

  // Configure BLE scan settings
  bleScan->setAdvertisedDeviceCallbacks(new ControllerBleAdvertisedDeviceCallbacks());
  bleScan->setActiveScan(true);
  bleScan->setInterval(100);
  bleScan->setWindow(99);

  // Until a device is connected...
  while (!isBleDeviceConnected) {
    // Scan for 5 seconds, trying to find BLE devices
    BLEScanResults foundDevices = bleScan->start(kScanTime_s, false);
    const int numFoundDevices = foundDevices.getCount();
    Serial.printf("Devices found: %i\n\n", numFoundDevices);

    // For every BLE device found during the scan...
    for (int i = 0; i < numFoundDevices; i++) {
      // If the name is "DSD TECH" (the manufacturer of the HM-10 BLE module we use)...
      if (foundDevices.getDevice(i).getName().compare("DSD TECH") == 0) {
        // Connect to it!
        Serial.println("== HM10 MODULE FOUND ==");
        bleClient->connect(foundDevices.getDevice(i).getAddress());
        Serial.println("== HM10 MODULE CONNECTED ==");
        break;
      } else {
        // Otherwise, the found device isn't the one we're looking for. Ignore it.
      }
    }

    // Reset the scan and wait 2s before continuing or trying again
    bleScan->clearResults();
    delay(kScanDelay_ms);
  }
}

void loop() {
  while (true) {
    uint16_t encodedButtons = 0xFFFF;
    uint8_t encodedJoysticks[4] = {127, 127, 127, 127};
    uint8_t blePayload[sizeof(encodedButtons) + sizeof(encodedJoysticks)];

    // Read buttons
    for (int i = 0; i < kPins_Buttons.size(); i++) {
      uint16_t readValue = digitalRead(kPins_Buttons[i]);
      encodedButtons ^= readValue << i;
    }

    // Read joysticks
    for (int i = 0; i < kPins_Joysticks.size(); i++) {
      uint32_t readValue = analogRead(kPins_Joysticks[i]);
      encodedJoysticks[i] = map(readValue, 0, 4095, 0, 254);
    }

    // Swap button encoding to big-endian
    encodedButtons = __htons(encodedButtons);

    // Combine inputs into payload
    memcpy(&blePayload[0], &encodedButtons, sizeof(encodedButtons));
    memcpy(&blePayload[sizeof(encodedButtons)], encodedJoysticks, sizeof(encodedJoysticks));

    // Log and transmit the value to the robot
    Serial.printf("TX: [0x%02x][0x%02x][0x%02x][0x%02x][0x%02x][0x%02x]\n", blePayload[0],
                  blePayload[1], blePayload[2], blePayload[3], blePayload[4], blePayload[5]);
    bleClient->getService(kBleServiceUUID)
        ->getCharacteristic(kBleCharUUID)
        ->writeValue(blePayload, sizeof(blePayload));

    // Delay to limit the transmission frequency
    delay(kLoopDelay_ms);
  }
}