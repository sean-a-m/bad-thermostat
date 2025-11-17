#include "BluetoothManager.h"

#include <Arduino.h>

namespace {
constexpr char SWITCHBOT_MAC_ADDRESS[] = "C5:38:32:30:16:1D";
constexpr char SERVICE_UUID[] = "cba20d00-224d-11e6-9fb8-0002a5d5c51b";
constexpr char RX_CHAR_UUID[] = "cba20002-224d-11e6-9fb8-0002a5d5c51b";
constexpr char NOTIFY_CHAR_UUID[] = "cba20003-224d-11e6-9fb8-0002a5d5c51b";

constexpr uint32_t SCAN_INTERVAL = 1349;
constexpr uint32_t SCAN_WINDOW = 449;
constexpr uint32_t SCAN_DURATION_SECONDS = 5;
constexpr uint32_t CONNECT_TIMEOUT_MS = 10000;
constexpr uint32_t CONNECT_RETRY_DELAY_MS = 250;
}  // namespace

BluetoothManager::BluetoothManager()
    : targetAddress_(SWITCHBOT_MAC_ADDRESS),
      serviceUUID_(SERVICE_UUID),
      rxCharUUID_(RX_CHAR_UUID),
      notifyCharUUID_(NOTIFY_CHAR_UUID),
      pClient_(nullptr),
      clientCallback_(nullptr),
      pRemoteCharacteristic_(nullptr),
      pNotifyCharacteristic_(nullptr),
      myDevice_(nullptr),
      pBLEScan_(nullptr),
      doConnect_(false),
      doReconnect_(false),
      connected_(false) {}

void BluetoothManager::initialize() {
    BLEDevice::init("");

    pBLEScan_ = BLEDevice::getScan();
    pBLEScan_->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks(*this));
    pBLEScan_->setInterval(SCAN_INTERVAL);
    pBLEScan_->setWindow(SCAN_WINDOW);
    pBLEScan_->setActiveScan(true);
    startScan(SCAN_DURATION_SECONDS);
}

void BluetoothManager::maintainConnection() {
    if (doConnect_) {
        if (connectToServer()) {
            Serial.println("Now connected to server");
        } else {
            Serial.println("Failed to connect to server!");
        }
        doConnect_ = false;
    }

    if (!connected_ && doReconnect_) {
        startScan(30);
        delay(1000);
    }
}

bool BluetoothManager::sendMessage() {
    if (!connected_) {
        Serial.println("Can't send message");
        return false;
    }

    if (pRemoteCharacteristic_ == nullptr) {
        Serial.println("Remote characteristic not initialized");
        return false;
    }

    Serial.println("Sending message");
    uint8_t cmd[] = {0x57, 0x1, 0x0};
    pRemoteCharacteristic_->writeValue(cmd, sizeof(cmd));
    return true;
}

void BluetoothManager::notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                                      uint8_t *pData,
                                      size_t length,
                                      bool isNotify) {
    Serial.print("Received data: ");
    for (size_t i = 0; i < length; ++i) {
        Serial.printf("0x%02X ", pData[i]);
    }
    Serial.println();
}

void BluetoothManager::ClientCallback::onConnect(BLEClient *pclient) {
    Serial.println("Connected to SwitchBot");
    manager_.connected_ = true;
}

void BluetoothManager::ClientCallback::onDisconnect(BLEClient *pclient) {
    Serial.println("Disconnected from SwitchBot");
    manager_.connected_ = false;
    manager_.doReconnect_ = true;
}

void BluetoothManager::AdvertisedDeviceCallbacks::onResult(BLEAdvertisedDevice advertisedDevice) {
    manager_.onDeviceFound(advertisedDevice);
}

void BluetoothManager::onDeviceFound(BLEAdvertisedDevice advertisedDevice) {
    BLEAddress target(targetAddress_);
    if (!advertisedDevice.getAddress().equals(target)) {
        return;
    }

    BLEDevice::getScan()->stop();
    Serial.println("Found target SwitchBot device");

    delete myDevice_;
    myDevice_ = new BLEAdvertisedDevice(advertisedDevice);
    doConnect_ = true;
    doReconnect_ = false;
}

bool BluetoothManager::connectToServer() {
    if (myDevice_ == nullptr) {
        Serial.println("No device to connect to");
        return false;
    }

    if (pClient_ == nullptr) {
        pClient_ = BLEDevice::createClient();
    }

    if (clientCallback_ == nullptr) {
        clientCallback_ = new ClientCallback(*this);
        pClient_->setClientCallbacks(clientCallback_);
    }

    const uint32_t connect_start = millis();
    bool is_connected = false;
    while (millis() - connect_start < CONNECT_TIMEOUT_MS) {
        if (pClient_->isConnected()) {
            is_connected = true;
            break;
        }

        Serial.println("Attempting BLE connection...");
        if (pClient_->connect(myDevice_)) {
            is_connected = true;
            break;
        }

        delay(CONNECT_RETRY_DELAY_MS);
        yield();
    }

    if (!is_connected) {
        Serial.println("BLE connection attempt timed out; will retry later");
        pClient_->disconnect();
        doReconnect_ = true;
        return false;
    }

    Serial.println("Connected to server");
    pClient_->setMTU(517);

    BLERemoteService *pRemoteService = pClient_->getService(serviceUUID_);
    if (pRemoteService == nullptr) {
        Serial.println("Failed to find service");
        return false;
    }
    Serial.println("Found service");

    pRemoteCharacteristic_ = pRemoteService->getCharacteristic(rxCharUUID_);
    if (pRemoteCharacteristic_ == nullptr) {
        Serial.println("Failed to find characteristic UUID");
        pClient_->disconnect();
        return false;
    }
    Serial.println("Found characteristic");

    if (pRemoteCharacteristic_->canRead()) {
        std::string value = pRemoteCharacteristic_->readValue();
        Serial.println("Characteristic value is: ");
        Serial.println(value.c_str());
    }

    if (pRemoteCharacteristic_->canNotify()) {
        pRemoteCharacteristic_->registerForNotify(notifyCallback);
    }

    pNotifyCharacteristic_ = pRemoteService->getCharacteristic(notifyCharUUID_);
    if (pNotifyCharacteristic_ == nullptr) {
        Serial.println("Failed to find notification characteristic UUID");
        pClient_->disconnect();
        return false;
    }
    Serial.println("Found notification characteristic");

    if (pNotifyCharacteristic_->canNotify()) {
        pNotifyCharacteristic_->registerForNotify(notifyCallback);
    }

    connected_ = true;
    return true;
}

void BluetoothManager::startScan(uint32_t durationSeconds) {
    if (pBLEScan_ == nullptr) {
        return;
    }

    pBLEScan_->start(durationSeconds, false);
}
