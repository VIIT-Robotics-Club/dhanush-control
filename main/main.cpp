#include <stdio.h>
#include "urosHandler.hpp"

#include "qmd.hpp"
#include "ballHandler/ballHandler.hpp"
#include "pinMap.hpp"
#include "quadrature.hpp"
#include "speedController.hpp"

qmd* handler = 0;
urosHandler* uros = 0;
decoder* dec = 0;
int pwmPins[] = {FLYWL_U_PWM, FLYWL_L_PWM, ARM_MOTOR_PWM, FLYW_ANGLE_L, FLYW_ANGLE_R};
int dirPins[] = {FLYWL_U_DIR, FLYWL_L_DIR, ARM_MOTOR_DIR, G_FLYW_ANGLE_DIR, G_FLYW_ANGLE_DIR};
int PhaseA[] = {FLY_UPPER_PHASE_A, FLY_LOWER_PHASE_A ,ARM_Phase_A};
int PhaseB[] = {GPIO_NUM_0,GPIO_NUM_0,ARM_Phase_B};
extern "C" void app_main(void)
{
    // initialize quad motoer
    handler = new qmd(pwmPins,dirPins,5);
    handler->setRange(19990, 0);
    handler->setRange(2500, 500, 3);
    handler->setRange(500, 2500, 4);

    dec = new decoder(PhaseA,PhaseB,3);

    urosHandler::config cfg = {
        .mode = urosHandler::config::TRANSPORT_USB
    };
    
    uros = new urosHandler(cfg);
    

    pid_config_t arm_pid = defPositionController;


    // configuration for ball handler
    ball_handler_config_t ball_handle = {
        .qmd_handler = handler,
        .decoder_handle = dec,
        .armController = positionController(arm_pid)
        // .flylController = speedController(handler,dec,3),
        // .flyuController = speedController(handler,dec,4)
    };

    ball_handle.armController.error_tolerance = 50.0f;
    // ball_handle.flyuController.error_tolerance = 10000.0f;
    // ball_handle.flylController.error_tolerance = 10000.0f;


    uros->addThreadExecutor({
        new ballHandler(ball_handle)     
    },PRO_CPU_NUM);
}
