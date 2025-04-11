#include "stdio.h"
#include "speedController.hpp"
#include "qmd.hpp"
#include "quadrature.hpp"
#include "pinMap.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

qmd* handler = 0;
decoder* dec = 0;
positionController* control = 0;
int pwmPins[] = {FLYWL_U_PWM, FLYWL_L_PWM, ARM_MOTOR_PWM, FLYW_ANGLE_L, FLYW_ANGLE_R};
int dirPins[] = {FLYWL_U_DIR, FLYWL_L_DIR, ARM_MOTOR_DIR, G_FLYW_ANGLE_DIR, G_FLYW_ANGLE_DIR};

int PhaseA[] = {ARM_Phase_A};
int PhaseB[] = {ARM_Phase_B};

extern "C" void app_main(void){

    handler = new qmd(pwmPins,dirPins,3);
    handler->setRange(19990, 0);

    dec = new decoder(PhaseA,PhaseB,1);

    control = new positionController(handler,dec,0);

    control->p = 0.03 , control->i = 0.0004 , control ->d = 0.00099;
    control->position = 500 , control->tolerance = 0.05;
    control->iRange = 10;
    while(1){
        dec->update();
        control->update();
        // ESP_LOGD("SPEED" ,"Speeds : %f",handler->speeds[0]);
        // handler->speeds[0] = 0.1;
        handler->update();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
