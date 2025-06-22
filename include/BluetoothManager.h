//
// Created by Tebbe on 23.04.2025.
//

#ifndef BLUETOOTHMANAGER_H
#define BLUETOOTHMANAGER_H
#include <BLEDevice.h>

#include "DataTypes.h"


class BluetoothManager
{

public:
    BluetoothManager(void (*vibrationCallback)(size_t size, const VibrationInterval_t *intervals));
    void streamIMUQuats(uint8_t (*pQuatData)[8]) const;
    void streamIMUQuats2(uint8_t (*pQuatData)[8]) const;

private:
    BLEServer* pServer;
    BLECharacteristic* pVibrationCharacteristic;
    BLECharacteristic* pIMUCharacteristic;
    void (*vibrationCallback)(size_t size, const VibrationInterval_t *intervals) = nullptr;

    void handleVibrationData(size_t size, uint8_t* pData) const;

    class ServerCallbacks : public BLEServerCallbacks
    {
    public:
        void onConnect(BLEServer* pServer) override;
        void onDisconnect(BLEServer* pServer) override;
    };

    class VibrationCharacteristicCallbacks : public BLECharacteristicCallbacks
    {
    public:
        VibrationCharacteristicCallbacks(BluetoothManager* manager) : manager(manager) {}
        void onWrite(BLECharacteristic* pCharacteristic) override;
    private:
        BluetoothManager* manager;
    };
};


#endif //BLUETOOTHMANAGER_H
