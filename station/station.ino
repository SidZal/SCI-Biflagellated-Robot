#include <ArduinoBLE.h>
#include "Joystick.h"

// Bluetooth Objects
BLEService orientationReceiver("b440");
BLECharacteristic angles("5a1e", BLEWrite, 12);
BLECharacteristic targets("6cee", BLERead | BLEWrite, 8);
BLECharacteristic rpm("ba89", BLEWrite, 8);
BLEDevice central;

// Joystick Objects
Joystick jsA(A0);
Joystick jsB(A1);
const int maxRPM = 80;

void setup() {
  Serial.begin(9600);
  while(!Serial);

  // Calibrate Joysticks
  Serial.println(jsA.calibrate());
  Serial.println(jsB.calibrate());
  delay(500);

  jsA.setMax(maxRPM);
  jsB.setMax(maxRPM);

  Serial.println("Initialzing BLE Service");  
  // Initialize BLE Service
  if(!BLE.begin()) {
    Serial.println("Failed to start BLE");
    while(1);
  }
  Serial.println("BLE Initialized");

  BLE.setLocalName("receiver");
  BLE.setAdvertisedService(orientationReceiver);

  orientationReceiver.addCharacteristic(angles);
  orientationReceiver.addCharacteristic(targets);
  orientationReceiver.addCharacteristic(rpm);

  BLE.addService(orientationReceiver);
  BLE.advertise();
  Serial.println("Setup Complete");
}

// Handles bluetooth, calls receiverLoop in normal operation
void loop() {
  // Conencted
  if(central.connected()) {
    receiverLoop();
  }
  // Recently Disconnected
  else if (central) {
    Serial.println("Disconnected from Central: " + central.address());
    central = BLE.central();
  }
  // Scanning
  else {
    Serial.println("Advertising BLE Service");
    while(!central.connected())
      central = BLE.central();
    Serial.println("Connected to Central: " + central.address());
  }
}

void receiverLoop() {
  // Write Joystick Inputs
  int jsIn[2];
  jsIn[0] = jsA.read()*maxRPM;
  jsIn[1] = jsB.read()*maxRPM;
  targets.writeValue(&jsIn, 8);

  if(angles.written()) {
    float orientation[3], speed[2];
    angles.readValue(&orientation, 12);
    rpm.readValue(&speed, 8);
    
    Serial.print(String(speed[0]) + ' ' + String(speed[1]));
    Serial.println(' ' + String(jsIn[0]) + ' ' + String(jsIn[1]) + ' ' + String(maxRPM) + ' ' + String(-maxRPM));
    Serial.println(String(orientation[0]) + ' ' + String(orientation[1]) +  ' ' + String(orientation[2]) + " 180 -180");
  }
}