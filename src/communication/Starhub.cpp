#pragma once
#include "src/communication/Starhub.h"

BLEServer* Starhub::server = NULL;
BLECharacteristic* Starhub::commandCharacteristic = NULL;
BLECharacteristic* Starhub::telemetryCharacteristic = NULL;
bool Starhub::connected = false;
bool Starhub::oldConnected = false;
