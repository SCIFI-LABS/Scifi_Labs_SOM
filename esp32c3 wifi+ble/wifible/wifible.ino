#include <WiFiManager.h>  
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// UUIDs for BLE service and characteristic
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

void setup() {
  // Initialize USB Serial for debugging
  Serial.begin(115200);
  
  // Initialize UART for STM32 communication (Serial1)
  Serial1.begin(115200, SERIAL_8N1, 21, 20);

  // WiFi setup using WiFiManager
  WiFi.mode(WIFI_STA);

  // Initialize WiFiManager
  WiFiManager wm;

  // Reset settings - for testing only; remove this in production
  wm.resetSettings();

  // Try to auto-connect to WiFi
  if (!wm.autoConnect("Scifi_LabsSOM", "pavanoscifilabs")) {
    Serial.println("Failed to connect to WiFi");
    // ESP.restart(); // Optionally restart or handle failure
  } else {
    Serial.println("Connected to WiFi successfully");
    // Send a message to STM32 over UART
    Serial1.println("ESP32C3 connected to WiFi and communicating over UART");
  }

  // BLE setup
  BLEDevice::init("Scifi_LabsSOM");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );
  pCharacteristic->setValue("Hello from Scifi_LabsSOM BLE Server");
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("Scifi_LabsSOM Server is now advertising.");
  
  // Send a message to STM32 over UART
  Serial1.println("ESP32C3 BLE server is now advertising");
}

void loop() {
  // You can periodically send data to STM32 over UART here
  delay(5000);
  Serial1.println("Periodic message from ESP32C3 to STM32 via UART");
}

