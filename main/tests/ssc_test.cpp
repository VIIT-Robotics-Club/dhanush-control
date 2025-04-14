
#include "speedController.hpp"
#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "SSC_TEST"

extern "C" void app_main(){


    smoothSpeedController controller;

    float output = 0.0f, feedback = 0.0f;

    controller.setIo(&output, &feedback);

    controller.speed = 10.0f;
    controller.a  = 1.0f;

    while (true)
    {
        ESP_LOGI(TAG, "speed %f target %f output %f reached %d", controller.speed, controller.targetOutput, controller.update(), controller.reached());

        if(controller.reached()) controller.speed = (controller.speed == 10.0f) ? 0.1f : 10.0f; 
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    

} ;