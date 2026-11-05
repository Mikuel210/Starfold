#pragma once
#ifndef STARFOLD_DATAPACKET_H
#define STARFOLD_DATAPACKET_H
#include "../sensors/SensorData.h"
#include "../hardware/HardwareData.h"
#include "../fusion/FusionData.h"
#include "../control/ControlData.h"
#include "../State.h"

struct DataPacket {
    State state;
    SensorData sensorData;
    HardwareData hardwareData;
    FusionData fusionData;
    ControlData controlData;
};

struct [[gnu::packed]] VectorBytes {
    float x, y, z;
};

struct [[gnu::packed]] PIDBytes {
    float p, i, d, r, s;
};

struct [[gnu::packed]] KFStateBytes {
    float position, velocity;
    float P11, P12, P22;
    float Q11, Q12, Q22;
};

struct [[gnu::packed]] PacketBytes {
    uint8_t state;

    // Bit 0: lidarAvailable
    // Bit 1: underVoltage
    uint8_t flags;

    // Sensor data
    VectorBytes accel_mps2;
    VectorBytes gyro_radps;
    VectorBytes magnetometer;
    float distance_cm;
    float voltage1_v;
    float voltage2_v;

    // Hardware data
    float tvc_deg[4];
    float leg_deg;
    int32_t throttle_us;

    // Fusion data
    VectorBytes orientation_deg;
    float altitude_mm;
    float correctedDistance_mm;
    float rocketAccelZ_mmps2;
    KFStateBytes altitudeKfState;

    // Control data
    VectorBytes correction_deg;
    float throttle_01;
    PIDBytes xPid_deg;
    PIDBytes yPid_deg;
    PIDBytes altitudePid_mm;
};

static_assert(sizeof(PacketBytes) == 206, "Unexpected telemetry packet layout");

#endif
