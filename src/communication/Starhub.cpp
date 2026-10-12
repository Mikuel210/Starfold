#pragma once
#include "Starhub.h"

BLEServer* Starhub::server = NULL;
BLECharacteristic* Starhub::commandCharacteristic = NULL;
BLECharacteristic* Starhub::telemetryCharacteristic = NULL;
bool Starhub::startupRequested = false;
bool Starhub::abortRequested = false;
bool Starhub::connected = false;
bool Starhub::oldConnected = false;
