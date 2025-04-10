#ifndef BALL_HANDLER_HPP
#define BALL_HANDLER_HPP


#include <qmd.hpp>
#include <quadrature.hpp>
#include <pinMap.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <urosElement.hpp>


#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <dhanush_srv/srv/speed_angle.h>
#include <std_msgs/msg/bool.h>

#include <multiThreadExecutor/workerThread.hpp>


// state of the mechanism represented as a object  
struct ball_handler_state_t {
    float arm_state = 0.0f, flyWheelSpeed = 0.0f, flywheel_angle = 0.0f;
    float feedbackFlyWheelSpeed_L = 0.0f, feedbackFlyWheelSpeed_U = 0.0f;

    float encoderFeedBack[DECODER_MAX_WHEEL_COUNT] = {0.0f};
    bool finger_state = false;
    // gpio levels of arm limiters
    bool armLimiterState[2] = {false};
};



/**
 * configuration to initialize a ballHandler
 * contains hardware mappings
 */
struct ball_handler_config_t{
    qmd* qmd_handler = 0;
    decoder* decoder_handle = 0;

    int flyWheelLower = INDEX_FLYW_L, 
        flyWheelUpper = INDEX_FLYW_U, 
        arm = INDEX_ARM, 
        flyWheelAngleLeft = INDEX_FLYW_ANGLE_L,
        flyWheelAngleRight = INDEX_FLYW_ANGLE_R;

    gpio_num_t armLimiterL = ARM_LIMIT_L, armLimiterU = ARM_LIMIT_U, finger = FINGER_GPIO;
};


struct threadContext_t {
    ball_handler_config_t* cfg;
    ball_handler_state_t* current, target;
};


class debugWorker : public workerThread {
public:
    
    inline void setContext(threadContext_t& p_ctx) { ctx = p_ctx;};
    void run();
    threadContext_t ctx;

    float alpha = 0.1f;
};

class ballHandler : public urosElement {

public:

    struct params_t : public urosElement::config
    {
        params_t() : urosElement::config("ballHandler", sizeof(params_t)) { load(); };
        bool debug = false;
    } params;
    

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

void declareDebugRclInterfaces();


    // debug subscription callback
    static void flyWheel_subs_callback(const void * msgin);
    static void arm_subs_callback(const void * msgin);
    static void service_callback(const void * req, void *res);
    static void angle_subs_callback(const void* msgin);
    static void finger_subs_callback(const void * msgin);

    void hw_task_callback();
    
    dhanush_srv__srv__SpeedAngle_Request req;
    dhanush_srv__srv__SpeedAngle_Response res;
    rcl_service_t service;


public:
    ball_handler_config_t cfg;
    rcl_subscription_t flyWheel_sub, arm_sub, angle_sub, finger_sub;
    static ballHandler* def;

    TaskHandle_t hwTaskHandle;
    ball_handler_state_t target_state;
    ball_handler_state_t current_state;

private:
    std_msgs__msg__Float32 flyWheel_msg , arm_msg, angle_msg;
    std_msgs__msg__Bool finger_msg;

    const rosidl_service_type_support_t  * support = ROSIDL_GET_SRV_TYPE_SUPPORT(dhanush_srv, srv, SpeedAngle);

private:
    // worker threads for multiple tasks
    debugWorker dbgWorker;
};

#endif // BALL_HANDLER_HPP