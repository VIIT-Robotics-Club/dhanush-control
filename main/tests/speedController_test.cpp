

#include <speedController.hpp>
#include <pinMap.hpp>
#include <quadrature.hpp>


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#define TAG "main"


int pwmPins[] = {FLYWL_U_PWM, FLYWL_L_PWM, ARM_MOTOR_PWM, FLYW_ANGLE_L, FLYW_ANGLE_R};
int dirPins[] = {FLYWL_U_DIR, FLYWL_L_DIR, ARM_MOTOR_DIR, G_FLYW_ANGLE_DIR, G_FLYW_ANGLE_DIR};
int PhaseA[] = {GPIO_NUM_36,GPIO_NUM_0,ARM_Phase_A};
int PhaseB[] = {GPIO_NUM_35,GPIO_NUM_0,ARM_Phase_B};


extern "C" void app_main(){

    qmd* handler = new qmd(pwmPins, dirPins, 1);

    handler->setRange(19990, 0);
    handler->setInvertingMode(false);

    decoder* dec = new decoder(PhaseA, PhaseB, 1);

    speedController::pid_config_t config = {
        .p = 0.001,
        .i = 0.015, 
        .d = 0.000135, 
        // .a = 0.0047,
        // .p = 0.0003f,
        // .i = 0.007f, 
        // .d = 0.0005f, 
        // .a = -0.015f,
        .dt = PID_DEFAULT_PERIOD_S, 
        .max = 1.0f, 
        .min = -1.0f, 
        .tolerance = 0.05f, 
        .iRange = 200.0f,
    };


    smoothSpeedController ccnt(config);

    ccnt.setIo(handler, dec, 0);
    ccnt.speed = 235.0;
    ccnt.error_tolerance = 0.0f;

    while (true){

        dec->update();
        ccnt.update();
        handler->update();

        ESP_LOGI(TAG, "ticks per sec %f pwm output %f", ccnt.feedbackSpeed, *ccnt.output);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

};