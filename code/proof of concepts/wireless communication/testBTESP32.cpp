#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID_CONTROL  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_UUID_STATUS "beb5483e-36e1-4688-b7f5-ea07361b26a9"

BLEServer* pServer = NULL;
BLECharacteristic* pCharControl = NULL;
bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    }
    
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        delay(500);
        pServer->getAdvertising()->start();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            Serial.println(value[0]); 
        }
    }
};

void setup() {
    Serial.begin(115200);
    
    BLEDevice::init("BLE_POC_ESP32");
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    pCharControl = pService->createCharacteristic(
        CHAR_UUID_CONTROL,
        BLECharacteristic::PROPERTY_WRITE
    );
    pCharControl->setCallbacks(new MyCallbacks());

    BLECharacteristic* pCharStatus = pService->createCharacteristic(
        CHAR_UUID_STATUS,
        BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharStatus->addDescriptor(new BLE2902());

    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    BLEDevice::startAdvertising();
}

void loop() {
    if (deviceConnected) {
        pCharControl->setValue("OK");
        pCharControl->notify(); 
        delay(2000); 
    }
    delay(10);
}