#pragma once
#ifndef STARFOLD_STARHUB_H
#define STARFOLD_STARHUB_H
#include "../Plotter.h"
#include "DataPacket.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Parameters
#define SERVICE_UUID "3343f2ee-4910-492a-b29b-181e051a631b"
#define COMMAND_UUID "313ce956-bbe0-47bf-8e32-b2c5e3a364ec"
#define TELEMETRY_UUID "cf0ce5fe-6f1a-41b4-88b6-1304a50a2776"

class Starhub {
    public:
        static bool startupRequested;
        static bool abortRequested;

        static void initialize() {
            BLEDevice::init("Starkit");

            // Create server
            server = BLEDevice::createServer();
            server->setCallbacks(new ServerCallbacks());

            // Create service
            BLEService *service = server->createService(SERVICE_UUID);

            // Create test characteristic
            commandCharacteristic = service->createCharacteristic(
                COMMAND_UUID,
                BLECharacteristic::PROPERTY_WRITE
            );

            telemetryCharacteristic = service->createCharacteristic(
                TELEMETRY_UUID,
                BLECharacteristic::PROPERTY_NOTIFY |
                BLECharacteristic::PROPERTY_READ
            );

            commandCharacteristic->setCallbacks(new CommandCallbacks());
            telemetryCharacteristic->addDescriptor(new BLE2902());

            // Start the service
            service->start();

            // Start advertising
            BLEAdvertising *advertising = BLEDevice::getAdvertising();
            advertising->addServiceUUID(SERVICE_UUID);
            advertising->setScanResponse(true);
            advertising->setMinPreferred(0x0);

            BLEDevice::startAdvertising();
            BLEDevice::setMTU(517);
            Plotter::log("Advertising...");
        }

        static void update() {
            // Connecting
            if (connected && !oldConnected) {
                oldConnected = connected;
                Plotter::log("Device Connected");
            }

            // Disconnecting
            if (!connected && oldConnected) {
                Plotter::log("Device Disconnected");
                delay(500);

                server->startAdvertising();
                Plotter::log("Advertising...");
                oldConnected = connected;
            }
        }

        static void notifyPacket(DataPacket packet)
        {
            if (!connected) return;
            PacketBytes bytes = serializePacket(packet);

            telemetryCharacteristic->setValue(
                reinterpret_cast<uint8_t*>(&bytes),
                sizeof(bytes)
            );

            telemetryCharacteristic->notify();
        }

    private:
        static BLEServer* server;
        static BLECharacteristic* commandCharacteristic;
        static BLECharacteristic* telemetryCharacteristic;

        static bool connected;
        static bool oldConnected;

        class ServerCallbacks: public BLEServerCallbacks {
            void onConnect(BLEServer* server) {
                connected = true;
            };

            void onDisconnect(BLEServer* server) {
                connected = false;
            }
        };

        class CommandCallbacks : public BLECharacteristicCallbacks {
            void onWrite(BLECharacteristic* characteristic) {
                String value = characteristic->getValue();

                if (value.length() == 0) {
                    Plotter::log("Received command length is 0");
                    return;
                }

                Plotter::log("Received command: " + value);

                // Interpret command
                if (value == "S")
                    startupRequested = true;
                if (value == "A")
                    abortRequested = true;
                else
                    Plotter::log("Unknown command: " + value);
            }
        };

        static PacketBytes serializePacket(const DataPacket& data) {
            const auto vector = [](const Vector3& v) -> VectorBytes {
                return {
                    static_cast<float>(v.x),
                    static_cast<float>(v.y),
                    static_cast<float>(v.z)
                };
            };

            const auto pid = [](const PIDData& p) -> PIDBytes {
                return {
                    static_cast<float>(p.p),
                    static_cast<float>(p.i),
                    static_cast<float>(p.d),
                    static_cast<float>(p.r),
                    static_cast<float>(p.s)
                };
            };

            // Initialize packet
            PacketBytes bytes = {};
            bytes.state = static_cast<uint8_t>(data.state);
            bytes.flags = (data.sensorData.lidarAvailable ? 0x01 : 0x00) | (data.fusionData.underVoltage ? 0x02 : 0x00);

            // Sensor data
            const auto& sensorData = data.sensorData;
            bytes.accel_mps2   = vector(sensorData.accel_mps2);
            bytes.gyro_radps   = vector(sensorData.gyro_radps);
            bytes.magnetometer = vector(sensorData.magnetometer);
            bytes.distance_cm  = static_cast<float>(sensorData.distance_cm);
            bytes.voltage1_v   = static_cast<float>(sensorData.voltage1_v);
            bytes.voltage2_v   = static_cast<float>(sensorData.voltage2_v);

            // Hardware data
            const auto& hardwareData = data.hardwareData;

            for (unsigned i = 0; i < 4; ++i) {
                bytes.tvc_deg[i] = static_cast<float>(hardwareData.tvc_deg[i]);
            }

            bytes.leg_deg = static_cast<float>(hardwareData.leg_deg);
            bytes.throttle_us = static_cast<int32_t>(hardwareData.throttle_us);

            // Fusion data
            const auto& fusionData = data.fusionData;
            bytes.orientation_deg = vector(fusionData.orientation_deg);
            bytes.altitude_mm = static_cast<float>(fusionData.altitude_mm);
            bytes.correctedDistance_mm = static_cast<float>(fusionData.correctedDistance_mm);
            bytes.rocketAccelZ_mmps2 = static_cast<float>(fusionData.rocketAccelZ_mmps2);

            const auto& kfState = fusionData.altitudeKfState;

            bytes.altitudeKfState = {
                static_cast<float>(kfState.position),
                static_cast<float>(kfState.velocity),
                static_cast<float>(kfState.P11),
                static_cast<float>(kfState.P12),
                static_cast<float>(kfState.P22),
                static_cast<float>(kfState.Q11),
                static_cast<float>(kfState.Q12),
                static_cast<float>(kfState.Q22)
            };

            // Control data
            const auto& controlData = data.controlData;
            bytes.correction_deg = vector(controlData.correction_deg);
            bytes.throttle_01    = static_cast<float>(controlData.throttle_01);
            bytes.xPid_deg       = pid(controlData.xPid_deg);
            bytes.yPid_deg       = pid(controlData.yPid_deg);
            bytes.altitudePid_mm = pid(controlData.altitudePid_mm);

            return bytes;
        }
};

#endif
