#pragma once
#include "IDataProvider.h"
#include "IHardwareProvider.h"
#include "Control.h"
#include "Fusion.h"
#include "Task.h"
#include "TaskArgs.h"

// BMS parameters
#define BMS_HZ 1

class BMS {
    public:
        static void initialize() {
            Task* task = new Task("bms", bmsTask);
            TaskArgs* args = new TaskArgs();
            args->task = task;
            task->start(args);
        }

    private:
        static void bmsTask(void* task) {
            TaskArgs* args = (TaskArgs*)task;

            while (true) {
                unsigned long msStart = millis();
                SensorData sensorData = Control::dataProvider->getData();
                FusionData fusionData = Fusion::getData(sensorData);

                if (fusionData.underVoltage) Control::hardwareProvider->writeBuzzer(HIGH);
                vTaskDelay(max(1000.0 / BMS_HZ / 2 - (millis() - msStart), 0.0));

                unsigned long msStop = millis();
                Control::hardwareProvider->writeBuzzer(LOW);
                vTaskDelay(max(1000.0 / BMS_HZ / 2 - (millis() - msStop), 0.0));
            }

            args->task->stop();
            delete args;
        }
};
