#ifndef BALL_HANDLER_HPP
#define BALL_HANDLER_HPP


#include <qmd.hpp>
#include <pinMap.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <urosElement.hpp>


#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <dhanush_srv/srv/speed_angle.h>
#include <std_msgs/msg/bool.h>



/**
 * configuration to initialize a ballHandler
 * contains hardware mappings
 */
struct ball_handler_config_t{
    qmd* qmd_handler = 0;

    int flyWheelLower = INDEX_FLYW_L, 
        flyWheelUpper = INDEX_FLYW_U, 
        arm = INDEX_ARM, 
        flyWheelAngleLeft = INDEX_FLYW_ANGLE_L,
        flyWheelAngleRight = INDEX_FLYW_ANGLE_R;

    gpio_num_t armLimiterL = ARM_LIMIT_L, armLimiterU = ARM_LIMIT_U, finger = FINGER_GPIO;
};


struct ball_handler_state_t {
    float arm_state = 0.0f, flyWheelSpeed = 0.0f, flywheel_angle = 0.0f;
    bool finger_state = false;
    
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
    
private:
        // subscription callback
    static void flyWheel_subs_callback(const void * msgin);
    static void arm_subs_callback(const void * msgin);
    static void service_callback(const void * req, void *res);
    static void angle_subs_callback(const void* msgin);
    static void finger_subs_callback(const void * msgin);

    void ballHandlerTask();
    
    dhanush_srv__srv__SpeedAngle_Request req;
    dhanush_srv__srv__SpeedAngle_Response res;
    rcl_service_t service;


public:
    ball_handler_config_t cfg;
    rcl_subscription_t flyWheel_sub, arm_sub, angle_sub, finger_sub;
    static ballHandler* def;

    TaskHandle_t taskHandle;
    ball_handler_state_t target_state;
    ball_handler_state_t current_state;

private:
    std_msgs__msg__Float32 flyWheel_msg , arm_msg, angle_msg;
    std_msgs__msg__Bool finger_msg;

    const rosidl_service_type_support_t  * support = ROSIDL_GET_SRV_TYPE_SUPPORT(dhanush_srv, srv, SpeedAngle);
// private:
};

#endif // BALL_HANDLER_HPP