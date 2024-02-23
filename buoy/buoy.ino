#include <ArduinoBLE.h>

// Bluetooth UUIDs
#define PERIPHERAL_UUID "b440"
#define ANGLES_UUID "5a1e"
#define TARGETS_UUID "6cee"
#define RPM_UUID "ba89"

// Bluetooth Helper Variables
bool scanning = false;
BLEDevice peripheral;

BLECharacteristic angles;
BLECharacteristic targets;
BLECharacteristic rpm;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  Serial.println("Initializing BLE");
  if(!BLE.begin()) {
    Serial.println("Failed to initialize BLE");
    while(1);
  }
  Serial.println("BLE Initialized");
}

void loop() {
  if(peripheral.connected()) {
    transmissionLoop();
  }
  else {

    if(!scanning) {
      BLE.scanForUuid(PERIPHERAL_UUID);
      scanning = true;
      Serial.println("Scanning...");
    }

    peripheral = BLE.available();
    if(peripheral) {
      Serial.println("Found: " + peripheral.localName());
      if (peripheral.localName() != "receiver") return;
      BLE.stopScan();
      scanning = false;

      // Verify Connection
      if(!peripheral.connect()) {
        Serial.println("Failed to Connect");
        return;
      }

      if(!peripheral.discoverAttributes()) {
        Serial.println("Failed to Discover Attributes");
        peripheral.disconnect();
        return;
      }

      verifyCharacteristic(&angles, ANGLES_UUID);
      verifyCharacteristic(&targets, TARGETS_UUID);
      verifyCharacteristic(&rpm, RPM_UUID);

      Serial.println("Connection Successful");
    }
  }
}

// Function to loop when connected
void transmissionLoop() {
  // Read targets from BT, write to Serial
  int target[2];
  targets.readValue(&target, 8);

  String serialTargets = String(target[0]) + "," + String(target[1]);
  Serial.println("Writing to Serial: " + serialTargets);
  Serial1.println(serialTargets);

  // Read orientation and motor RPMs from Serial, write to BT
  if(Serial1.available()) { 
    float orient[3];
    float v[2];

    readFloatArray(orient, 3);
    readFloatArray(v, 2);

    Serial1.readStringUntil('\n');

    Serial.print("Writing to BT: "); 
    printArray(orient, 3);
    printArray(v, 2);
    Serial.println();

    angles.writeValue(&orient, 12);
    rpm.writeValue(&v, 8);
  }
}

// Shorthand function to verify BLE Characteristics
void verifyCharacteristic(BLECharacteristic *characteristic, char* UUID) {
  *characteristic = peripheral.characteristic(UUID);
  if (!*characteristic) {
    Serial.println("No Characteristic for UUID " + String(UUID));
    peripheral.disconnect();
    return;
  } else if (!characteristic->canWrite()) {
    Serial.println("Cannot Write to Characteristic w/ UUID " + String(UUID));
    peripheral.disconnect();
    return;
  }
}

void readFloatArray(float *arr, int length) {
  for(int i = 0; i<length; i++)
    arr[i] = Serial1.readStringUntil(',').toFloat();
}

void printArray(float *arr, int length) {
  for(int i=0; i<length; i++)
    Serial.print(String(arr[i]) + ',');
}

