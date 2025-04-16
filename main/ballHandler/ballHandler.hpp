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
#include <std_srvs/srv/trigger.h>
#include <std_msgs/msg/bool.h>

#include <multiThreadExecutor/workerThread.hpp>
#include <speedController.hpp>


// TODO convert these to ros2 parameters
// configuration for arm
#define ARM_REST_POS 1000.0     // arm position of rest in terms of encoder ticks 
#define ARM_IN_POS 3000.0       // inwards arm position  in terms of encoder ticks 
#define ARM_OUT_POS 0.0         // outwards arm position  in terms of encoder ticks 

// configuration for dribble
#define GRIPPER_OFF_TO_FINGER_MS 100    // delay in ms, betweem gripper release and finger active
#define FINGER_WAIT_MS 100              // delay in ms, duration for finger to be active
#define GRAB_DELAY_MS 100               // delay in ms, time for ball to reach gripper

#define PRE_LAUNCH_DELAY_MS 3000

// state of the mechanism represented as a object  
struct ball_handler_state_t {
    float arm_state = 0.0f, flyWheelSpeed_L = 0.0f, flyWheelSpeed_U = 0.0f, flywheel_angle = 0.7f;
    float feedbackFlyWheelSpeed_L = 0.0f, feedbackFlyWheelSpeed_U = 0.0f;
    float flyWheelSpeed = 0.0f;

    float encoderFeedBack[DECODER_MAX_WHEEL_COUNT] = {0.0f};
    bool finger_state = false, gripper_state = false;
    // gpio levels of arm limiters, exterior limiter and interior limiter
    bool armLimiterState[2] = {false};

    enum launch_state_t {
        LAUNCH_BEGIN = 0,
        LAUNCH_PRE_THROW,
        LAUNCH_POST_THROW,
        LAUNCH_COMPLETE,
    } launchState = LAUNCH_BEGIN;

    enum dribble_state_t {
        DRIBBLE_BEGIN = 0,
        DRIBBLE_READY,
        DRIBBLE_PRE_THROW,
        DRIBBLE_POST_THROW,
        DRIBBLE_COMPLETE,
    } dribbleState = DRIBBLE_BEGIN;
};



/**
 * configuration to initialize a ballHandler
 * contains hardware mappings
 */
struct ball_handler_config_t{
    qmd* qmd_handler = 0;
    decoder* decoder_handle = 0;
    positionController armController;
    smoothSpeedController flylController, flyuController;

    int flyWheelLower = INDEX_FLYW_L, 
        flyWheelUpper = INDEX_FLYW_U, 
        arm = INDEX_ARM, 
        flyWheelAngleLeft = INDEX_FLYW_ANGLE_L,
        flyWheelAngleRight = INDEX_FLYW_ANGLE_R;

    gpio_num_t armLimiterExterior = ARM_LIMIT_L, armLimiterInterior = ARM_LIMIT_U, finger = FINGER_GPIO;
    gpio_num_t gripper = GRIPPER_GPIO;
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


class launchWorker : public workerThread {
public:

    inline void setContext(threadContext_t& p_ctx) { ctx = p_ctx;};
    void run();
    threadContext_t ctx;

    float alpha = 0.1f;
};


class dribbleWorker : public workerThread {
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

void declareDebugRclInterfaces();


    // debug subscription callback
    static void flyWheel_subs_callback(const void * msgin);
    static void arm_subs_callback(const void * msgin);
    static void service_callback(const void * req, void *res);
    static void dribble_service_callback(const void * req, void *res);
    static void angle_subs_callback(const void* msgin);
    static void finger_subs_callback(const void * msgin);

    void hw_task_callback();
    
    dhanush_srv__srv__SpeedAngle_Request req;
    dhanush_srv__srv__SpeedAngle_Response res;
    std_srvs__srv__Trigger_Request  dribble_req;
    std_srvs__srv__Trigger_Response dribble_res;

    rcl_service_t service, dribble_service;


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
    const rosidl_service_type_support_t  * trigger_support = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger);

private:
    // worker threads for multiple tasks
    debugWorker dbgWorker;
    launchWorker lnchWorker;
    dribbleWorker drbWorker;


    enum eventType {
        THROW_SERVICE = 0,
        DRIBBLE_SERVICE,
        DEBUG_EVENT,
    };

    void eventInput(eventType type);
};

#endif // BALL_HANDLER_HPP