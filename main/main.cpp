#include <stdio.h>
#include "urosHandler.hpp"

#include "qmd.hpp"
#include "ballHandler/ballHandler.hpp"
#include "pinMap.hpp"


qmd* handler = 0;
urosHandler* uros = 0;


int pwmPins[] = {FLYWL_U_PWM, FLYWL_L_PWM, ARM_MOTOR_PWM, FLYW_ANGLE_L, FLYW_ANGLE_R};
int dirPins[] = {FLYWL_U_DIR, FLYWL_L_DIR, ARM_MOTOR_DIR, G_FLYW_ANGLE_DIR, G_FLYW_ANGLE_DIR};

extern "C" void app_main(void)
{
    // initialize quad motoer
    handler = new qmd(pwmPins,dirPins,5);
    handler->setRange(19990, 0);

    urosHandler::config cfg = {
        .mode = urosHandler::config::TRANSPORT_USB
    };
    
    uros = new urosHandler(cfg);
    
    // configuration for ball handler
    ball_handler_config_t ball_handle = {
        .qmd_handler = handler,
    };


    uros->addThreadExecutor({
        new ballHandler(ball_handle)     
    },PRO_CPU_NUM);
        
}
