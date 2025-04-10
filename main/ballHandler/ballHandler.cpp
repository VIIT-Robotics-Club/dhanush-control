
#include "ballHandler.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include <esp_log.h>

#include <urosHandler.hpp>

#define TAG "ball_handler"

#define BALL_HANDLER_TASK_NAME "ballHandlerTask"
#define BALL_HANDLER_TASK_STACK 4096
#define BALL_HANDLER_TASK_PRIORITY 10

const char  *flyWheel_topic_name = "/flyWheel_speed", 
            *arm_topic_name = "/arm_speed",
            *finger_topic_name = "/finger",
            *angle_topic_name = "/flywheel_angle";

const char * service_name = "launch_ball";


const rosidl_message_type_support_t * float32_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);
const rosidl_message_type_support_t * bool_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);


ballHandler* ballHandler::def = 0;

void arm_limiter_isr(void * arg){
    ballHandler* handler = (ballHandler*) arg;

    // TODO change to update qmd directly, status flag
    handler->current_state.arm_state = 0.0f;
    vTaskNotifyGiveFromISR(handler->hwTaskHandle, NULL);
};



ballHandler::ballHandler(ball_handler_config_t& p_cfg) : cfg(p_cfg){

    // worker threads setup 
    threadContext_t ctx = {
        .cfg = &cfg,
        .current = &current_state
    };

    dbgWorker.setContext(ctx);
    dbgWorker.start();




    def = this;
 //starts ball handler thread
    xTaskCreate((TaskFunction_t) &ballHandler::hw_task_callback, BALL_HANDLER_TASK_NAME, BALL_HANDLER_TASK_STACK, 
        this, BALL_HANDLER_TASK_PRIORITY, &hwTaskHandle);


//maps ISR to arm limiter
    gpio_config_t limiter_cfg = {
        .pin_bit_mask = (uint64_t)((0x01 << cfg.armLimiterL) | (0x01 << cfg.armLimiterU)),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };

    gpio_config_t finger_cfg = {
        .pin_bit_mask = (uint64_t)(0x01 << cfg.finger),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };

    gpio_config(&limiter_cfg);
    gpio_config(&finger_cfg);
    gpio_install_isr_service( ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED);

    gpio_isr_handler_add(cfg.armLimiterU, arm_limiter_isr,  this);
    gpio_isr_handler_add(cfg.armLimiterL, arm_limiter_isr, this);
    
    // Enable the interrupts
    gpio_intr_enable(cfg.armLimiterU);
    gpio_intr_enable(cfg.armLimiterL);



};



void ballHandler::declareDebugRclInterfaces(){
    std_msgs__msg__Float32__init(&flyWheel_msg);
    std_msgs__msg__Float32__init(&arm_msg);
    std_msgs__msg__Float32__init(&angle_msg);
    std_msgs__msg__Bool__init(&finger_msg);

    ESP_ERROR_CHECK(rclc_subscription_init_default(&flyWheel_sub, node, float32_type_support, flyWheel_topic_name));
    ESP_ERROR_CHECK(rclc_subscription_init_default(&arm_sub, node, float32_type_support, arm_topic_name));
    rclc_subscription_init_default(&finger_sub, node, bool_type_support, finger_topic_name);
    ESP_ERROR_CHECK(rclc_subscription_init_default(&angle_sub, node, float32_type_support, angle_topic_name));

    
    rclc_executor_add_subscription(exec, &flyWheel_sub, &flyWheel_msg, flyWheel_subs_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(exec, &arm_sub, &arm_msg, arm_subs_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(exec, &finger_sub, &finger_msg, finger_subs_callback, ON_NEW_DATA);
    ESP_ERROR_CHECK(rclc_executor_add_subscription(exec, &angle_sub, &angle_msg,angle_subs_callback, ON_NEW_DATA));
};

void ballHandler::init(){
    if(params.debug) declareDebugRclInterfaces();

    dhanush_srv__srv__SpeedAngle_Request__init(&req);
    dhanush_srv__srv__SpeedAngle_Response__init(&res);

    // create launch service to trigger ball launch sequence  
    ESP_ERROR_CHECK(rclc_service_init_default(&service, node, support, service_name));
    ESP_ERROR_CHECK(rclc_executor_add_service(exec, &service, &req, &res, service_callback));
};


void ballHandler::declareParameters(){
    urosHandler::addParameter_bool("ballHandler_debug", &params.debug, &params);
};



void ballHandler::hw_task_callback(){

    while (1){
        // get gpio levels
        current_state.armLimiterState[0] = gpio_get_level(cfg.armLimiterL);
        current_state.armLimiterState[1] = gpio_get_level(cfg.armLimiterU);

        // update gpio levels
        gpio_set_level(cfg.finger, current_state.finger_state);

        // calculate speeds from encoder ticks
        cfg.decoder_handle->update();
        current_state.feedbackFlyWheelSpeed_L = cfg.decoder_handle->count[cfg.flyWheelLower] - current_state.encoderFeedBack[cfg.flyWheelLower];
        current_state.feedbackFlyWheelSpeed_U = cfg.decoder_handle->count[cfg.flyWheelUpper] - current_state.encoderFeedBack[cfg.flyWheelUpper];

        // copy current encoder state
        memcpy(&current_state.encoderFeedBack, cfg.decoder_handle->count, DECODER_MAX_WHEEL_COUNT * sizeof(float));
        
        //  speed 
        cfg.qmd_handler->speeds[cfg.flyWheelLower] = cfg.qmd_handler->speeds[cfg.flyWheelUpper] = current_state.flyWheelSpeed;
        cfg.qmd_handler->speeds[cfg.flyWheelAngleLeft] = cfg.qmd_handler->speeds[cfg.flyWheelAngleRight] = current_state.flywheel_angle;
        cfg.qmd_handler->speeds[cfg.arm] = current_state.arm_state;
        cfg.qmd_handler->update();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    vTaskDelete(NULL);
};

void ballHandler::flyWheel_subs_callback(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    if(def) def->current_state.flyWheelSpeed = msg->data;

    ESP_LOGD(TAG, "flyWheel state : %f", msg->data);
}

void ballHandler::arm_subs_callback(const void* msgin){
    
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    if(def) def->current_state.arm_state = msg->data;

}

void ballHandler::angle_subs_callback(const void* msgin){
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32*)msgin;

    // memcpy(msg->data, &mapped, sizeof(float));
    if(def && msg->data <= 1.0f && msg->data >= 0.0f) def->current_state.flywheel_angle = msg->data;
}

void ballHandler::service_callback(const void * req, void *res)
{
    dhanush_srv__srv__SpeedAngle_Request * req_in = (dhanush_srv__srv__SpeedAngle_Request*)req;
    dhanush_srv__srv__SpeedAngle_Response * res_in = (dhanush_srv__srv__SpeedAngle_Response*)res;

    ESP_LOGI(TAG, "%s service called with angle %lf speed %lf", service_name, req_in->angle, req_in->speed);    

    res_in->success = true;
}


void ballHandler::finger_subs_callback(const void* msgin){
    
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    if(def) def->current_state.finger_state = msg->data;

    ESP_LOGD(TAG, "finger state: %d", msg->data);
}



// Dubug worker accesses all the indexes of qmd
void debugWorker::run(){

    while (true){
        queryRunningState();

        // handle all the inputs from debug topic
        ctx.current->flyWheelSpeed  = (1.0 - alpha) * ctx.current->flyWheelSpeed + (alpha)  * ctx.target.flyWheelSpeed;
        ctx.current->finger_state = ctx.target.finger_state;
        ctx.current->flywheel_angle = ctx.target.flywheel_angle;


        // TODO: PID implementation of arm state
        // ctx.current->arm_state

        ESP_LOGI("debugWorker", "working");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
};