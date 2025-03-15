

#include <esp_log.h>
#include <urosHandler.hpp>




#define TAG "main"

#include <std_msgs/msg/int32.h>
#include "pinMap.hpp"
#include "ballHandler/ballHandler.hpp"

#include <qmd.hpp>
 
const rosidl_message_type_support_t* int32_typ_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

class testNode : public urosElement {

public:
    testNode(){
        active = this;
    };


    void init(){
        std_msgs__msg__Int32__init(&msg);
        msg.data = 0;
        rclc_publisher_init_default(&publisher, node, int32_typ_support, "int32_pub");

        rclc_timer_init_default(&timer, support, RCL_MS_TO_NS(500), rcl_timer_callback);
        rclc_executor_add_timer(exec, &timer);

    };


    void declareParameters(){

    };

    static void rcl_timer_callback(rcl_timer_t *, int64_t){
        rcl_publish(&active->publisher, &active->msg, NULL);
        active->msg.data++;
    };

private:
    rcl_publisher_t publisher;
    rcl_timer_t timer;
    std_msgs__msg__Int32 msg;

    static testNode* active;
};

testNode* testNode::active = nullptr;


int pwm_pins[] = {GPIO_NUM_17 }; 
int dir_pins[] = {GPIO_NUM_18 };


qmd* drivers = 0;
extern "C" void app_main(){

    drivers = new qmd(pwm_pins, dir_pins, 1);
    drivers->setInvertingMode(false);
    drivers->setRange(19990, 0);

    drivers->speeds[0] = 1.0f;
    drivers->update();

    
    ball_handler_config_t cfg = {
        .qmd_handler = drivers,
        .flyWheelLower = 0,
        .flyWheelUpper = 0,
    };

    ballHandler bHandler(cfg);
    bHandler.current_state.arm_state = 1.0f;
    bHandler.current_state.flyWheelSpeed = 1.0f;

    urosHandler* uros = new urosHandler();


    uros->addThreadExecutor({
        new testNode()
    }, APP_CPU_NUM);
    

    // uros.

    while (1)
    {

        // ESP_LOGI(TAG, "hello from main");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
};