#include "BluetoothManager.h"

#include <Arduino.h>
#include <esp_task_wdt.h>

namespace {
// constexpr char SWITCHBOT_MAC_ADDRESS[] = "C5:38:32:30:16:1D";
constexpr char SWITCHBOT_MAC_ADDRESS[] = "C6:38:32:30:28:81";
constexpr char SERVICE_UUID[] = "cba20d00-224d-11e6-9fb8-0002a5d5c51b";
constexpr char RX_CHAR_UUID[] = "cba20002-224d-11e6-9fb8-0002a5d5c51b";
constexpr char NOTIFY_CHAR_UUID[] = "cba20003-224d-11e6-9fb8-0002a5d5c51b";

// Scan almost continuously so a slowly advertising SwitchBot isn't missed.
constexpr uint32_t SCAN_INTERVAL = 100;
constexpr uint32_t SCAN_WINDOW = 99;
constexpr uint32_t SCAN_DURATION_SECONDS = 8;
constexpr uint32_t SCAN_STOP_SETTLE_MS = 100;
constexpr int CONNECT_ATTEMPTS = 3;
constexpr uint32_t CONNECT_RETRY_DELAY_MS = 250;
// How long to keep the link up after writing the press, so it isn't dropped
// before the SwitchBot has acted on it. Ends early when it responds.
constexpr uint32_t RESPONSE_WAIT_MS = 1500;
constexpr uint32_t DISCONNECT_WAIT_MS = 2000;
constexpr uint32_t DISCOVERY_WAIT_MS = 5000;

// Exponential backoff bounds for delivery retries. Starts at MIN after the
// first failure and doubles up to MAX, so an unreachable SwitchBot is retried
// at a sane cadence instead of every loop iteration.
constexpr uint32_t CONNECTION_BACKOFF_MIN_MS = 3000;
constexpr uint32_t CONNECTION_BACKOFF_MAX_MS = 60000;

// The BLE stack discovers the SwitchBot's services by itself right after
// connecting. Asking for them before that has finished makes the stack run
// our request from deep inside its own discovery handling, which overflows
// the Bluetooth task's stack and panics.
volatile bool discovery_complete = false;

void onGattcEvent(esp_gattc_cb_event_t event, esp_gatt_if_t, esp_ble_gattc_cb_param_t *) {
    if (event == ESP_GATTC_DIS_SRVC_CMPL_EVT) {
        discovery_complete = true;
    }
}
}  // namespace

BluetoothManager::BluetoothManager()
    : targetAddress_(SWITCHBOT_MAC_ADDRESS),
      serviceUUID_(SERVICE_UUID),
      rxCharUUID_(RX_CHAR_UUID),
      notifyCharUUID_(NOTIFY_CHAR_UUID),
      pClient_(nullptr),
      myDevice_(nullptr),
      pBLEScan_(nullptr),
      deviceFound_(false),
      responseReceived_(false),
      lastConnectionAttemptTick_(0),
      connectionBackoffMs_(0),
      consecutiveConnectionFailures_(0) {}

bool BluetoothManager::beginConnectionRetryIfDue() {
    if (millis() - lastConnectionAttemptTick_ < connectionBackoffMs_) {
        return false;
    }
    lastConnectionAttemptTick_ = millis();
    return true;
}

void BluetoothManager::noteConnectionSuccess() {
    consecutiveConnectionFailures_ = 0;
    connectionBackoffMs_ = CONNECTION_BACKOFF_MIN_MS;
}

void BluetoothManager::noteConnectionFailure() {
    ++consecutiveConnectionFailures_;
    if (connectionBackoffMs_ < CONNECTION_BACKOFF_MIN_MS) {
        connectionBackoffMs_ = CONNECTION_BACKOFF_MIN_MS;
    } else if (connectionBackoffMs_ < CONNECTION_BACKOFF_MAX_MS / 2) {
        connectionBackoffMs_ *= 2;
    } else {
        connectionBackoffMs_ = CONNECTION_BACKOFF_MAX_MS;
    }
}

void BluetoothManager::initialize() {
    BLEDevice::init("");
    BLEDevice::setCustomGattcHandler(onGattcEvent);

    pBLEScan_ = BLEDevice::getScan();
    pBLEScan_->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks(*this));
    pBLEScan_->setInterval(SCAN_INTERVAL);
    pBLEScan_->setWindow(SCAN_WINDOW);
    pBLEScan_->setActiveScan(true);

    pClient_ = BLEDevice::createClient();
    pClient_->setClientCallbacks(&clientCallback_);
}

bool BluetoothManager::deliverPress() {
    if (pBLEScan_ == nullptr || pClient_ == nullptr) {
        Serial.println("BLE not initialized");
        return false;
    }

    // A link left over from an attempt that failed part way.
    if (pClient_->isConnected()) {
        disconnect();
    }

    esp_task_wdt_reset();
    if (!findDevice()) {
        Serial.println("SwitchBot not found in scan");
        return false;
    }

    if (!connect()) {
        Serial.println("Failed to connect to SwitchBot");
        return false;
    }

    esp_task_wdt_reset();
    bool sent = writePress();
    disconnect();
    esp_task_wdt_reset();
    return sent;
}

// Scanning before every connection confirms the SwitchBot is in range right
// now, so the connect that follows doesn't sit waiting on an absent device,
// and gives the address type the connection needs.
bool BluetoothManager::findDevice() {
    deviceFound_ = false;
    Serial.println("Scanning for SwitchBot...");
    if (!pBLEScan_->start(SCAN_DURATION_SECONDS, nullptr, false)) {
        Serial.println("Failed to start BLE scan");
        return false;
    }

    const uint32_t scan_start = millis();
    while (!deviceFound_ && millis() - scan_start < SCAN_DURATION_SECONDS * 1000UL) {
        esp_task_wdt_reset();
        delay(20);
    }
    pBLEScan_->stop();
    // Let a result that was mid-delivery finish before the device is used.
    delay(SCAN_STOP_SETTLE_MS);
    esp_task_wdt_reset();
    return deviceFound_ && myDevice_ != nullptr;
}

bool BluetoothManager::connect() {
    for (int attempt = 0; attempt < CONNECT_ATTEMPTS; ++attempt) {
        esp_task_wdt_reset();
        Serial.println("Attempting BLE connection...");
        discovery_complete = false;
        if (pClient_->connect(myDevice_)) {
            return true;
        }
        delay(CONNECT_RETRY_DELAY_MS);
    }
    return false;
}

bool BluetoothManager::writePress() {
    const uint32_t discovery_start = millis();
    while (!discovery_complete && pClient_->isConnected() &&
           millis() - discovery_start < DISCOVERY_WAIT_MS) {
        esp_task_wdt_reset();
        delay(20);
    }
    if (!pClient_->isConnected()) {
        Serial.println("Disconnected during service discovery");
        return false;
    }
    if (!discovery_complete) {
        Serial.println("Service discovery did not finish in time");
        return false;
    }

    // Reload on every connection: the client keeps the services (and their
    // characteristics) from the previous one otherwise.
    pClient_->getServices();
    BLERemoteService *service = pClient_->getService(serviceUUID_);
    if (service == nullptr) {
        Serial.println("Failed to find service");
        return false;
    }

    BLERemoteCharacteristic *rx = service->getCharacteristic(rxCharUUID_);
    if (rx == nullptr) {
        Serial.println("Failed to find characteristic UUID");
        return false;
    }

    // The response is only used to end the wait early and for the log.
    responseReceived_ = false;
    BLERemoteCharacteristic *notify = service->getCharacteristic(notifyCharUUID_);
    if (notify != nullptr && notify->canNotify()) {
        notify->registerForNotify(
            [this](BLERemoteCharacteristic *, uint8_t *pData, size_t length, bool) {
                Serial.print("Received data: ");
                for (size_t i = 0; i < length; i++) {
                    Serial.printf("0x%02X ", pData[i]);
                }
                Serial.println();
                responseReceived_ = true;
            });
    }

    esp_task_wdt_reset();
    if (!pClient_->isConnected()) {
        Serial.println("Disconnected before the press was sent");
        return false;
    }

    Serial.println("Sending message");
    uint8_t cmd[] = {0x57, 0x1, 0x0};
    rx->writeValue(cmd, sizeof(cmd));

    const uint32_t wait_start = millis();
    while (!responseReceived_ && pClient_->isConnected() &&
           millis() - wait_start < RESPONSE_WAIT_MS) {
        delay(20);
    }
    if (responseReceived_) {
        return true;
    }
    if (!pClient_->isConnected()) {
        Serial.println("Disconnected before the SwitchBot responded");
        return false;
    }
    Serial.println("No response from SwitchBot; assuming the press was delivered");
    return true;
}

void BluetoothManager::disconnect() {
    if (!pClient_->isConnected()) {
        return;
    }
    pClient_->disconnect();
    const uint32_t wait_start = millis();
    while (pClient_->isConnected() && millis() - wait_start < DISCONNECT_WAIT_MS) {
        delay(20);
    }
}

void BluetoothManager::ClientCallback::onConnect(BLEClient *pclient) {
    Serial.println("Connected to SwitchBot");
}

void BluetoothManager::ClientCallback::onDisconnect(BLEClient *pclient) {
    Serial.println("Disconnected from SwitchBot");
}

void BluetoothManager::AdvertisedDeviceCallbacks::onResult(BLEAdvertisedDevice advertisedDevice) {
    manager_.onDeviceFound(advertisedDevice);
}

void BluetoothManager::onDeviceFound(BLEAdvertisedDevice advertisedDevice) {
    if (deviceFound_ || !advertisedDevice.getAddress().equals(targetAddress_)) {
        return;
    }

    // Runs on the BLE task. The scan is stopped by findDevice(), which is
    // polling deviceFound_.
    Serial.println("Found target SwitchBot device");
    delete myDevice_;
    myDevice_ = new BLEAdvertisedDevice(advertisedDevice);
    deviceFound_ = true;
}
