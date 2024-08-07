#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define SERVICE_UUID                              "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define VALUE_CHARACTERISTIC_UUID                 "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define AUTOUPDATE_CHARACTERISTIC_UUID            "f751d216-4044-461f-a4e6-548314d60679"
#define TARGET_VALUE_CHARACTERISTIC_UUID          "44e7fcba-4db0-4c41-b0a8-a34fc66afd74"
#define TARGET_VALUE_PROGRESS_CHARACTERISTIC_UUID "6d913149-d0fc-4d85-9dd5-615248f2bee0"

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;     

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

BLECharacteristic *pValueCharacteristic;
BLECharacteristic *pAutoupdateCharacteristic;
BLECharacteristic *pTargetValueCharacteristic;
BLECharacteristic *pTargetValueProgressCharacteristic;

void getUpdatedValue(float temperatureC) {
  pValueCharacteristic->setValue(temperatureC);
  pValueCharacteristic->notify();
}

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    sensors.requestTemperatures(); 
    float temperatureC = sensors.getTempCByIndex(0);
    getUpdatedValue(temperatureC);
    BLEDevice::stopAdvertising();
  };
  void onDisconnect(BLEServer* pServer) {
    BLEDevice::startAdvertising();
  }
};

enum TargeValueProgress: std::uint8_t {
  notInProgress = 0,
  onTheWay = 1,
  onPoint = 2,
  passedThePoint = 3
};

bool autoUpdatesSending = false;
uint32_t autoUpdate = 2000;
bool targetValueActive = false;
float targetValue = 0.0;
float targetValueDelta = 1.0;
TargeValueProgress targetValueProgress = TargeValueProgress::notInProgress;

class CharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    size_t dataLength = pCharacteristic->getLength();
    uint8_t *data = pCharacteristic->getData();;
    std::__cxx11::string uuid = pCharacteristic->getUUID().toString();

    if (uuid == AUTOUPDATE_CHARACTERISTIC_UUID) {
      if (dataLength == 4) {
        autoUpdate = data[0] |  data[1] << 8 | data[2] << 16 | data[3] << 24;
        autoUpdatesSending = true;
        Serial.print("Auto update interval: ");
        Serial.print(autoUpdate);
        Serial.print('\n');
      } else {
        Serial.println("Auto update disabled");
        autoUpdatesSending = false;
      }
    } else if (uuid == TARGET_VALUE_CHARACTERISTIC_UUID) {
      if (dataLength == 4) {
        float *temp = (float *) data;
        targetValue = *temp;
        targetValueActive = true;
        targetValueProgress = TargeValueProgress::onTheWay;
        Serial.print("New target value: ");
        Serial.print(targetValue);
        Serial.print(' ');
        Serial.print((uint8_t) targetValueProgress);
        Serial.print('\n');
      } else {
        targetValueProgress = TargeValueProgress::notInProgress;
        targetValueActive = false;
        Serial.println("Target value disabled");
      }
      pTargetValueProgressCharacteristic->setValue((uint8_t *) &targetValueProgress, 1);
      pTargetValueProgressCharacteristic->notify();
    }
  }
};

void checkTargetValue(float temperatureC) {
  if (temperatureC >= targetValue - targetValueDelta && 
      temperatureC <= targetValue + targetValueDelta
  ) {
    if (targetValueProgress != TargeValueProgress::onPoint) {
      targetValueProgress = TargeValueProgress::onPoint;
      pTargetValueProgressCharacteristic->setValue((uint8_t *) &targetValueProgress, 1);
      pTargetValueProgressCharacteristic->notify();
      Serial.println("Target value reached");
    }
  } else if (targetValueProgress == TargeValueProgress::onPoint) {
    targetValueProgress = TargeValueProgress::passedThePoint;
    pTargetValueProgressCharacteristic->setValue((uint8_t *) &targetValueProgress, 1);
    pTargetValueProgressCharacteristic->notify();
    Serial.println("Target value passed");
  }
}

void setup() {
  Serial.begin(115200);

  // Start the DS18B20 sensor
  Serial.println("Starting DS18B20 sensor!");
  sensors.begin();


  // Start the BLE
  Serial.println("Starting BLE work!");

  BLEDevice::init("MyESP32");

  BLEServer *pServer = BLEDevice::createServer();

  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pValueCharacteristic = pService->createCharacteristic(
                                         VALUE_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );

  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);

  pValueCharacteristic->setValue(temperatureC);

  pAutoupdateCharacteristic = pService->createCharacteristic(
                                         AUTOUPDATE_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  if (autoUpdatesSending) {
    pAutoupdateCharacteristic->setValue(autoUpdate);
  } else {
    pAutoupdateCharacteristic->setValue(nullptr, 0);
  }

  pTargetValueCharacteristic = pService->createCharacteristic(
                                         TARGET_VALUE_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  if (targetValueActive) {
    pTargetValueCharacteristic->setValue(targetValue);
  } else {
    pTargetValueCharacteristic->setValue(nullptr, 0);
  }

  pTargetValueProgressCharacteristic = pService->createCharacteristic(
                                         TARGET_VALUE_PROGRESS_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );

  pTargetValueProgressCharacteristic->setValue((uint8_t *) &targetValueProgress, 1);

  CharacteristicCallbacks *characteristicCallbacks = new CharacteristicCallbacks();
  pAutoupdateCharacteristic->setCallbacks(characteristicCallbacks);
  pTargetValueCharacteristic->setCallbacks(characteristicCallbacks);

  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);

  if (autoUpdatesSending) {
    getUpdatedValue(temperatureC);
  }
  if (targetValueActive) {
    checkTargetValue(temperatureC);
  }

  Serial.print(temperatureC);
  Serial.println("°C");

  delay(autoUpdatesSending ? autoUpdate : 1000);
}