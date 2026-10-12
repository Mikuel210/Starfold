#pragma once
#include <Arduino.h>

class Task {
    public:
        volatile bool* finished = new bool(false);
        volatile bool* requestStop = new bool(false);

        Task(const char* name_, void (*function_)(void*))
            : name(name_), function(function_) {}

        void await() {
            while (!(*finished))
                delay(1);
        }

        void start(void* arg = NULL) {
            xTaskCreatePinnedToCore(function, name, 10000, arg, 1, &handle, 1);
        }

        void stop() {
            if (handle == NULL) return;

            *finished = true;
            vTaskDelete(handle);
            handle = NULL;
        }

    private:
        TaskHandle_t handle = NULL;
        const char* name;
        void (*function)(void*);
};
