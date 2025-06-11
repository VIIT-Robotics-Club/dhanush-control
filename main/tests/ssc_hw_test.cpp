
#include "speedController.hpp"
#include <esp_log.h>
#include "qmd.hpp"
#include "pinMap.hpp"
#include "quadrature.hpp"
#include "speedController.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "SSC_HW_TEST"

qmd* handler = 0;
decoder* dec = 0;

int pwmPins[] = {FLYWL_U_PWM};
int dirPins[] = {FLYWL_U_DIR};
int PhaseA[] = {FLY_UPPER_PHASE_A};
int PhaseB[] = {GPIO_NUM_0,GPIO_NUM_0,ARM_Phase_B};

extern "C" void app_main(){

    handler = new qmd(pwmPins,dirPins,5);       //Object for motor 
    handler->setRange(19990, 0);                
    handler->speeds[0] = 0.5;
    handler->update();

    dec = new decoder(PhaseA,PhaseB,3);         //Object for encoder

    smoothSpeedController controller;           //SpeedController Object
    controller.setIo(handler, dec, 0);          
    controller.speed = 10.0f;

    while (true)
    {
        dec->update();
        ESP_LOGI(TAG, "speed %f target %f output %f reached %d", controller.speed, 
                controller.targetOutput, controller.update(), controller.reached());
        handler->update();

        if(controller.reached()) controller.speed = (controller.speed == 10.0f) ? 0.1f : 10.0f;         
        vTaskDelay(pdMS_TO_TICKS(100));
    }
} ;