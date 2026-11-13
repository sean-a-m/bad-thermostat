#pragma once

#include <BLEDevice.h>

// Talks to the SwitchBot only when there is a press to deliver: scan for it,
// connect, write the press command, disconnect. Nothing is held open between
// presses, and nothing depends on a scan at boot having succeeded.
class BluetoothManager {
public:
    BluetoothManager();

    void initialize();

    // Scan, connect, send one press and disconnect. Blocks for several
    // seconds. Returns true when the press was written to the SwitchBot.
    bool deliverPress();

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
        void onConnect(BLEClient *pclient) override;
        void onDisconnect(BLEClient *pclient) override;
    };

    class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    public:
        explicit AdvertisedDeviceCallbacks(BluetoothManager &manager) : manager_(manager) {}
        void onResult(BLEAdvertisedDevice advertisedDevice) override;

    private:
        BluetoothManager &manager_;
    };

    void onDeviceFound(BLEAdvertisedDevice advertisedDevice);
    bool findDevice();
    bool connect();
    bool writePress();
    void disconnect();

    BLEAddress targetAddress_;
    BLEUUID serviceUUID_;
    BLEUUID rxCharUUID_;
    BLEUUID notifyCharUUID_;

    BLEClient *pClient_;
    ClientCallback clientCallback_;
    BLEAdvertisedDevice *myDevice_;
    BLEScan *pBLEScan_;

    // Set from the BLE task.
    volatile bool deviceFound_;
    volatile bool responseReceived_;

    uint32_t lastConnectionAttemptTick_;
    uint32_t connectionBackoffMs_;
    uint32_t consecutiveConnectionFailures_;
};
