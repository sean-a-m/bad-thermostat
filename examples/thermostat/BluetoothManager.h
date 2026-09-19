#pragma once

#include <BLEDevice.h>

class BluetoothManager {
public:
    BluetoothManager();

    void initialize();
    void maintainConnection();
    bool sendMessage();
    bool isConnected() const { return connected_; }

    // Backoff-gated retry: returns true (and stamps the attempt time) only
    // once the current backoff interval has elapsed. Caller should treat a
    // true return as "you may attempt BLE work this iteration".
    bool beginConnectionRetryIfDue();
    // Reset failure counter and backoff after a successful delivery.
    void noteConnectionSuccess();
    // Record a failed attempt: grows the backoff and the failure counter.
    void noteConnectionFailure();
    uint32_t consecutiveConnectionFailures() const { return consecutiveConnectionFailures_; }

private:
    class ClientCallback : public BLEClientCallbacks {
    public:
        explicit ClientCallback(BluetoothManager &manager) : manager_(manager) {}
        void onConnect(BLEClient *pclient) override;
        void onDisconnect(BLEClient *pclient) override;

    private:
        BluetoothManager &manager_;
    };

    class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    public:
        explicit AdvertisedDeviceCallbacks(BluetoothManager &manager) : manager_(manager) {}
        void onResult(BLEAdvertisedDevice advertisedDevice) override;

    private:
        BluetoothManager &manager_;
    };

    static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                               uint8_t *pData,
                               size_t length,
                               bool isNotify);

    void onDeviceFound(BLEAdvertisedDevice advertisedDevice);
    bool connectToServer();
    void startScan(uint32_t durationSeconds);

    BLEAddress targetAddress_;
    BLEUUID serviceUUID_;
    BLEUUID rxCharUUID_;
    BLEUUID notifyCharUUID_;

    BLEClient *pClient_;
    ClientCallback *clientCallback_;
    BLERemoteCharacteristic *pRemoteCharacteristic_;
    BLERemoteCharacteristic *pNotifyCharacteristic_;
    BLEAdvertisedDevice *myDevice_;
    BLEScan *pBLEScan_;

    bool doConnect_;
    bool doReconnect_;
    bool connected_;

    uint32_t lastConnectionAttemptTick_;
    uint32_t connectionBackoffMs_;
    uint32_t consecutiveConnectionFailures_;
};
