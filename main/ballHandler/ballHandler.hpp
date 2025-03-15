#ifndef BALL_HANDLER_HPP
#define BALL_HANDLER_HPP


#include <qmd.hpp>
#include <pinMap.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <urosElement.hpp>


/**
 * configuration to initialize a ballHandler
 * contains hardware mappings
 */
struct ball_handler_config_t{
    qmd* qmd_handler = 0;
    int flyWheelLower = INDEX_FLYW_L, flyWheelUpper = INDEX_FLYW_U, arm = INDEX_ARM;
    gpio_num_t armLimiterL = ARM_LIMIT_L, armLimiterU = ARM_LIMIT_U;
};


struct ball_handler_state_t {
    float arm_state = 0.0f, flyWheelSpeed = 0.0f;
    
};

class ballHandler : public urosElement {

public:

    /**
     * constructor to initialize ball handler
     * initializes internal data
     * starts ball handler thread
     * maps ISR to arm limiter
     */ 
    ballHandler(ball_handler_config_t& cfg);
    

    void declareParameters();
    void init();



    
private:

    void ballHandlerTask();

public:
    ball_handler_config_t cfg;
    TaskHandle_t taskHandle;
    ball_handler_state_t target_state;
    ball_handler_state_t current_state;

// private:
};

#endif // BALL_HANDLER_HPP