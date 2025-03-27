
#include "ballHandler.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include <esp_log.h>

#define TAG "ball_handler"

#define BALL_HANDLER_TASK_NAME "ballHandlerTask"
#define BALL_HANDLER_TASK_STACK 4096
#define BALL_HANDLER_TASK_PRIORITY 10

const char * flyWheel_topic_name = "/flyWheel_speed", *arm_topic_name = "/arm_speed", *angle_topic_name = "/flywheel_angle";

const rosidl_message_type_support_t * float32_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);

ballHandler* ballHandler::def = 0;

void arm_limiter_isr(void * arg){
    ballHandler* handler = (ballHandler*) arg;

    handler->current_state.arm_state = 0.0f;
    vTaskNotifyGiveFromISR(handler->taskHandle, NULL);
};



ballHandler::ballHandler(ball_handler_config_t& p_cfg) : cfg(p_cfg){

    def = this;
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


    
    
    // res.message.data = &resMsg;
    // res.message.size = 1;
    // res.message.capacity = 1;
    
    // dhanush_srv__srv__SpeedAngle_Request__init(&req);
    // ESP_ERROR_CHECK(dhanush_srv__srv__SpeedAngle_Response__init(&res));

    gpio_isr_handler_add(cfg.armLimiterU, arm_limiter_isr,  this);
    gpio_isr_handler_add(cfg.armLimiterL, arm_limiter_isr, this);

    // Enable the interrupts
    gpio_intr_enable(cfg.armLimiterU);
    gpio_intr_enable(cfg.armLimiterL);

// initializes internal data


};



void ballHandler::init(){
    std_msgs__msg__Float32__init(&flyWheel_msg);
    std_msgs__msg__Float32__init(&arm_msg);
    std_msgs__msg__Float32__init(&angle_msg);


    ESP_ERROR_CHECK(rclc_subscription_init_default(&flyWheel_sub, node, float32_type_support, flyWheel_topic_name));
    ESP_ERROR_CHECK(rclc_subscription_init_default(&arm_sub, node, float32_type_support, arm_topic_name));
    ESP_ERROR_CHECK(rclc_subscription_init_default(&angle_sub, node, float32_type_support, angle_topic_name));
    
    rclc_executor_add_subscription(exec, &flyWheel_sub, &flyWheel_msg, flyWheel_subs_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(exec, &arm_sub, &arm_msg, arm_subs_callback, ON_NEW_DATA);
    ESP_ERROR_CHECK(rclc_executor_add_subscription(exec, &angle_sub, &angle_msg,angle_subs_callback, ON_NEW_DATA));

    // ESP_ERROR_CHECK(rclc_service_init_default(&service, node, support, service_name));
    // ESP_ERROR_CHECK(rclc_executor_add_service(exec, &service, &req, &res, service_callback));
    // printf("Hello World");
};

void ballHandler::declareParameters(){

};


void ballHandler::ballHandlerTask(){

    while (1){

        // check for service calls

        // update motor speeds
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

    printf("Service request value: %f + %f.\n",req_in->angle,  req_in->speed);
}

