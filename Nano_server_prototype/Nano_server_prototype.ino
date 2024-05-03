/*
  Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
  Ported to Arduino ESP32 by Evandro Copercini
  updated by chegewara and MoThunderz
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>

double number = 0;            // your double
byte* data = (byte*)&number;  // pointer to the
volatile bool dataReceived = false;  // flag to indicate data has been received


// Parameters for the sensor:

// Initialize all pointers
BLEServer* pServer = NULL;                    // Pointer to the server
BLECharacteristic* pCharacteristic_1 = NULL;  // Pointer to Characteristic 1
BLECharacteristic* pCharacteristic_2 = NULL;  // Pointer to Characteristic 2
BLEDescriptor* pDescr_1;                      // Pointer to Descriptor of Characteristic 1
BLE2902* pBLE2902_1;                          // Pointer to BLE2902 of Characteristic 1
BLE2902* pBLE2902_2;                          // Pointer to BLE2902 of Characteristic 2

// Some variables to keep track on device connected
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Variable that will continuously be increased and written to the client
uint32_t value = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
// UUIDs used in this example:
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_1 "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_2 "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

// Callback function that is called whenever a client is connected or disconnected
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void setup() {
  Serial.begin(115200);
  Wire.begin(8);                 // join i2c bus with address #8
  Wire.onReceive(receiveEvent);  // register event

  // Configure the trigger pin to output mode
  //pinMode(TRIG_PIN, OUTPUT);
  // Configure the echo pin to input mode
  //pinMode(ECHO_PIN, INPUT);

  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic_1 = pService->createCharacteristic(
    CHARACTERISTIC_UUID_1,
    BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic_2 = pService->createCharacteristic(
    CHARACTERISTIC_UUID_2,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  // Create a BLE Descriptor
  pDescr_1 = new BLEDescriptor((uint16_t)0x2901);
  pDescr_1->setValue("A very interesting variable");
  pCharacteristic_1->addDescriptor(pDescr_1);

  // Add the BLE2902 Descriptor because we are using "PROPERTY_NOTIFY"
  pBLE2902_1 = new BLE2902();
  pBLE2902_1->setNotifications(true);
  pCharacteristic_1->addDescriptor(pBLE2902_1);

  pBLE2902_2 = new BLE2902();
  pBLE2902_2->setNotifications(true);
  pCharacteristic_2->addDescriptor(pBLE2902_2);

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {

  //Serial.println("I have entered the loop");
  // notify changed value
  if (deviceConnected) {
    Serial.print("I have entered the connect");
      Serial.println(number);  // print the received number
      String txValue = String(number);
      pCharacteristic_2->setValue(txValue.c_str());

      pCharacteristic_1->setValue(number);
      //pCharacteristic_1->notify();
      value++;

      std::string rxValue = pCharacteristic_2->getValue();
  }
  //Reset the flag
  // The code below keeps the connection status uptodate:
  // Disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // Connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}

void receive_data(double* array, int size) {
  if (Wire1.available()) {  // if data is available to read
    int index = 0;
    while (Wire1.available() && index < size) {
      byte data[sizeof(double)];  // to store incoming bytes
      for (int i = 0; i < sizeof(double); i++) {
        if (Wire1.available()) {
          data[i] = Wire1.read();  // read a byte
        }
      }
      double number = *(double*)&data;  // convert byte array to double
      array[index] = number;            // store the number in the array
      index++;
    }
  }
}

void receiveEvent(int howMany) {
  for (int i = 0; i < sizeof(double); i++) {
    data[i] = Wire.read();  // read one byte at a time
  }
}
