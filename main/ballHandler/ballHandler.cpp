
#include "ballHandler.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#define TAG "ball handler"

#define BALL_HANDLER_TASK_NAME "ballHandlerTask"
#define BALL_HANDLER_TASK_STACK 4096
#define BALL_HANDLER_TASK_PRIORITY 10


void arm_limiter_isr(void * arg){
    ballHandler* handler = (ballHandler*) arg;

    handler->current_state.arm_state = 0.0f;
    vTaskNotifyGiveFromISR(handler->taskHandle, NULL);
};



ballHandler::ballHandler(ball_handler_config_t& p_cfg) : cfg(p_cfg){


//starts ball handler thread
    xTaskCreate((TaskFunction_t) &ballHandler::ballHandlerTask, BALL_HANDLER_TASK_NAME, BALL_HANDLER_TASK_STACK, 
        this, BALL_HANDLER_TASK_PRIORITY, &taskHandle);

//maps ISR to arm limiter
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (uint64_t)((0x01 << cfg.armLimiterL) | (0x01 << cfg.armLimiterU)),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };

    gpio_config(&gpio_cfg);
    gpio_install_isr_service( ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED);
    gpio_isr_handler_add(cfg.armLimiterU, arm_limiter_isr,  this);
    gpio_isr_handler_add(cfg.armLimiterL, arm_limiter_isr, this);

    // Enable the interrupts
    gpio_intr_enable(cfg.armLimiterU);
    gpio_intr_enable(cfg.armLimiterL);

// initializes internal data


};



void ballHandler::init(){

};

void ballHandler::declareParameters(){

};


void ballHandler::ballHandlerTask(){

    while (1){

        // check for service calls

        // update motor speeds
        cfg.qmd_handler->speeds[cfg.flyWheelLower] = cfg.qmd_handler->speeds[cfg.flyWheelUpper] = current_state.flyWheelSpeed;
        cfg.qmd_handler->speeds[cfg.arm] = current_state.arm_state;
        cfg.qmd_handler->update();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    vTaskDelete(NULL);
};