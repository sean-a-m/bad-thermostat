#include <Arduino.h>
#include <TFT_eSPI.h>
#include "pin_config.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include "OneButton.h"

#define ONE_WIRE_BUS 21

// Setup OneWire instance
OneWire oneWire(ONE_WIRE_BUS);

// Pass OneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

TFT_eSPI tft = TFT_eSPI();


// SwitchBot MAC address (convert colons to your ESP32 format)
BLEAddress switchbotAddress("C5:38:32:30:16:1D");
static std::string SWITCHBOT_MAC_ADDRESS = "C5:38:32:30:16:1D";


BLEUUID serviceUUID("cba20d00-224d-11e6-9fb8-0002a5d5c51b");
BLEUUID rxCharUUID("cba20002-224d-11e6-9fb8-0002a5d5c51b");
BLEUUID notifyCharUUID("cba20003-224d-11e6-9fb8-0002a5d5c51b");

static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLERemoteCharacteristic* pNoteRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
static boolean doConnect = false;
static boolean doScan = false;

// Global BLE client
BLEClient* pClient = nullptr;
bool connected = false;

OneButton button1(PIN_BUTTON_1, true);

// Notification callback function
void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                    uint8_t* pData, 
                    size_t length, 
                    bool isNotify) {
  Serial.print("Received data: ");
  for (int i = 0; i < length; i++) {
    Serial.printf("0x%02X ", pData[i]);
  }
  Serial.println();
}

// Connection callback
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("Connected to SwitchBot");
    connected = true;
  }

  void onDisconnect(BLEClient* pclient) {
    Serial.println("Disconnected from SwitchBot");
    connected = false;
  }
};

bool connectToServer() {
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());

    pClient->connect(myDevice);
    Serial.println("Connected to server");
    pClient->setMTU(517);

    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
        Serial.println("Failed to find service");
        return false;
    }
    Serial.println("Found service");

    pRemoteCharacteristic = pRemoteService->getCharacteristic(rxCharUUID);
    if (pRemoteCharacteristic == nullptr) {
        Serial.println("Failed to find characteristic UUID");
        pClient->disconnect();
        return false;
    }
    Serial.println("Found characteristic");
    
    if (pRemoteCharacteristic->canRead()) {
        std::string value = pRemoteCharacteristic->readValue();
        Serial.println("Characteristic value is: ");
        Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify()) {
        pRemoteCharacteristic->registerForNotify(notifyCallback);
    }

    pNoteRemoteCharacteristic = pRemoteService->getCharacteristic(notifyCharUUID);
    if (pNoteRemoteCharacteristic == nullptr) {
        Serial.println("Failed to find notification characteristic UUID");
        pClient->disconnect();
        return false;
    }
    Serial.println("Found notification characteristic");

    if (pNoteRemoteCharacteristic->canNotify()) {
        pNoteRemoteCharacteristic->registerForNotify(notifyCallback);
    }
    connected = true;
    return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
     //   Serial.println("advertised device found");
        BLEAddress target = BLEAddress(SWITCHBOT_MAC_ADDRESS);
        if (advertisedDevice.getAddress().equals(target)) {
            BLEDevice::getScan()->stop();
            Serial.println("found device");
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true;
            doScan = true;
        } else {
         //   Serial.println("Other Advertised device is");
        }
    }
};

bool pressButtonTwo() {
    if (connected) {
        if (pRemoteCharacteristic == nullptr) {
            Serial.println("Not initialized, returning");
            return false;
        }

        Serial.println("Sending message");
        uint8_t cmd[] = {0x57, 0x1, 0x0};
        pRemoteCharacteristic->writeValue(cmd, 3);
        // delay(10);
    } else {
        Serial.println("Can't send message");
      //   delay(1000);
    }
    return true;
}

void setup()
{
    Serial.begin(115200);
    sensors.begin();
    
    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);

    button1.attachClick([]() {
        pressButtonTwo();
    });

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(10, 10);
    tft.print("test");

    BLEDevice::init("");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(5, false);
    
    Serial.println("Thermostat project initialized");
}

void loop()
{
    if (doConnect == true) {
        if (connectToServer()) {
            Serial.println("Now connected to server");
        } else {
            Serial.println("Failed to connect to server!");
        }
        doConnect = false;
    }
   // sensors.requestTemperatures();
        // Get temperature in Celsius
 //   float temperatureC = sensors.getTempCByIndex(0);
    
    // Get temperature in Fahrenheit
 //   float temperatureF = sensors.getTempFByIndex(0);
    
    /*
    // Print results
    Serial.print("Temperature: ");
    Serial.print(temperatureC);
    Serial.print("°C / ");
    Serial.print(temperatureF);
    Serial.println("°F");
*/ 
    if (connected) {
        button1.tick();
    } else if (doScan) {
        BLEDevice::getScan()->start(0);
        delay(1000);
    }

  //  delay(1000);
}
