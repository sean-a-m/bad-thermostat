#pragma once

#include <BLEDevice.h>

class BluetoothManager {
public:
    BluetoothManager();

    void initialize();
    void maintainConnection();
    bool sendMessage();
    bool isConnected() const { return connected_; }

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
};
