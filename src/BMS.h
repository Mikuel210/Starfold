#pragma once
#ifndef STARFOLD_BMS_H
#define STARFOLD_BMS_H
#include "Globals.h"
#include "sensors/IDataProvider.h"
#include "hardware/IHardwareProvider.h"
#include "control/Control.h"
#include "fusion/Fusion.h"
#include "tasks/Task.h"
#include "tasks/TaskArgs.h"

class BMS {
    public:
        static void initialize() {
            #ifndef DISABLE_BMS
            Task* task = new Task("bms", bmsTask);
            TaskArgs* args = new TaskArgs();
            args->task = task;
            task->start(args);
            #endif
        }

    private:
        static void bmsTask(void* task) {
            TaskArgs* args = (TaskArgs*)task;
            double delay_ms = 1000.0 / BMS_HZ / 2;

            while (true) {
                unsigned long msStart = millis();
                SensorData sensorData = Control::dataProvider->getData();
                FusionData fusionData = Fusion::getVoltageData(sensorData);

                if (fusionData.underVoltage) Control::hardwareProvider->writeBuzzer(HIGH);
                vTaskDelay(max((delay_ms - (millis() - msStart)) / portTICK_PERIOD_MS, 0.0));

                unsigned long msStop = millis();
                Control::hardwareProvider->writeBuzzer(LOW);
                vTaskDelay(max((delay_ms - (millis() - msStop)) / portTICK_PERIOD_MS, 0.0));
            }

            args->task->stop();
            delete args;
        }
};

#endif
