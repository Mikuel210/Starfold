#pragma once
#ifndef STARFOLD_STARHUB_H
#define STARFOLD_STARHUB_H
#include "../Plotter.h"
#include "DataPacket.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

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

            JsonDocument document;
            document["state"] = static_cast<int>(packet.state);

            // Serialize sensor data
            JsonObject sensorData = document["sensorData"].to<JsonObject>();
            JsonArray acceleration = sensorData["accel_mps2"].to<JsonArray>();
            JsonArray gyroscope = sensorData["gyro_radps"].to<JsonArray>();
            JsonArray magnetometer = sensorData["magnetometer"].to<JsonArray>();

            acceleration.add(packet.sensorData.accel_mps2.x);
            acceleration.add(packet.sensorData.accel_mps2.y);
            acceleration.add(packet.sensorData.accel_mps2.z);

            gyroscope.add(packet.sensorData.gyro_radps.x);
            gyroscope.add(packet.sensorData.gyro_radps.y);
            gyroscope.add(packet.sensorData.gyro_radps.z);

            magnetometer.add(packet.sensorData.magnetometer.x);
            magnetometer.add(packet.sensorData.magnetometer.y);
            magnetometer.add(packet.sensorData.magnetometer.z);

            sensorData["distance_cm"] = packet.sensorData.distance_cm;
            sensorData["lidarAvailable"] = packet.sensorData.lidarAvailable;
            sensorData["voltage1_v"] = packet.sensorData.voltage1_v;
            sensorData["voltage2_v"] = packet.sensorData.voltage2_v;

            // Serialize fusion data
            JsonObject fusionData = document["fusionData"].to<JsonObject>();
            JsonArray orientation = fusionData["orientation_deg"].to<JsonArray>();

            orientation.add(packet.fusionData.orientation_deg.x);
            orientation.add(packet.fusionData.orientation_deg.y);
            orientation.add(packet.fusionData.orientation_deg.z);

            fusionData["altitude_mm"] = packet.fusionData.altitude_mm;
            fusionData["underVoltage"] = packet.fusionData.underVoltage;

            // Notify packet
            String output;
            serializeJson(document, output);

            telemetryCharacteristic->setValue(output);
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
};

#endif
