
#include "ballHandler.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include <esp_log.h>

#include <urosHandler.hpp>

#define TAG "ball_handler"
#define DBG_WRKR "debug worker"
#define LNCH_WRKR "launch worker"
#define DRIBL_WRKR "dribble worker"


#define BALL_HANDLER_TASK_NAME "ballHandlerTask"
#define BALL_HANDLER_TASK_STACK 4096
#define BALL_HANDLER_TASK_PRIORITY 10

const char  *flyWheel_topic_name = "/flyWheel_speed", 
            *arm_topic_name = "/arm_speed",
            *finger_topic_name = "/finger",
            *angle_topic_name = "/flywheel_angle";

const char * service_name = "launch_ball", *dribble_service_name = "dribble";


const rosidl_message_type_support_t * float32_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);
const rosidl_message_type_support_t * bool_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);


ballHandler* ballHandler::def = 0;
void arm_limiter_isr_ext(void * arg){
    ballHandler* handler = (ballHandler*) arg;
    ball_handler_config_t& cfg = handler->cfg;

    // TODO change to update qmd directly, status flag
    // handler->current_state.arm_state = 0.0f;
    cfg.qmd_handler->speeds[cfg.arm]  = 0.0f;
};

void arm_limiter_isr_int(void * arg){
    ballHandler* handler = (ballHandler*) arg;
    ball_handler_config_t& cfg = handler->cfg;

    // TODO change to update qmd directly, status flag
    cfg.qmd_handler->speeds[cfg.arm]  = 0.0f;
};


ballHandler::ballHandler(ball_handler_config_t& p_cfg) : cfg(p_cfg){


    cfg.armController.output = &current_state.arm_state;

    def = this;
    //starts ball handler thread
    xTaskCreate((TaskFunction_t) &ballHandler::hw_task_callback, BALL_HANDLER_TASK_NAME, BALL_HANDLER_TASK_STACK, 
        this, BALL_HANDLER_TASK_PRIORITY, &hwTaskHandle);

    // configure pid controllers
    cfg.armController.setIo(&current_state.arm_state, &cfg.decoder_handle->count[cfg.arm]);
    cfg.flylController.setIo(&current_state.flyWheelSpeed_L, &cfg.decoder_handle->count[cfg.flyWheelLower]);
    cfg.flyuController.setIo(&current_state.flyWheelSpeed_U, &cfg.decoder_handle->count[cfg.flyWheelUpper]);

//maps ISR to arm limiter
    gpio_config_t limiter_cfg = {
        .pin_bit_mask = (uint64_t)((0x01 << cfg.armLimiterExterior) | (0x01 << cfg.armLimiterInterior)),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };

    gpio_config_t arm_gpio_cfg = {
        .pin_bit_mask = (uint64_t)(0x01 << cfg.finger | 0x01 << cfg.gripper),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };

    gpio_config_t gripper_pir_config = {
        .pin_bit_mask = (uint64_t)(0x01 << cfg.gripper_pir),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };


    gpio_config(&limiter_cfg);
    gpio_config(&arm_gpio_cfg);
    gpio_config(&gripper_pir_config);

    gpio_install_isr_service( ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED);

    gpio_isr_handler_add(cfg.armLimiterInterior, arm_limiter_isr_int,  this);
    gpio_isr_handler_add(cfg.armLimiterExterior, arm_limiter_isr_ext, this);
    
    // Enable the interrupts
    gpio_intr_enable(cfg.armLimiterInterior);
    gpio_intr_enable(cfg.armLimiterExterior);
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

    // start worker threads
    threadContext_t ctx = {
        .cfg = &cfg,
        .current = &current_state
    };

    dbgWorker.setContext(ctx);
    dbgWorker.start();
    lnchWorker.setContext(ctx);
    lnchWorker.block();
    lnchWorker.start();
    drbWorker.setContext(ctx);
    drbWorker.block();
    drbWorker.start();
    

    dhanush_srv__srv__SpeedAngle_Request__init(&req);
    dhanush_srv__srv__SpeedAngle_Response__init(&res);
    std_srvs__srv__Trigger_Request__init(&dribble_req);
    std_srvs__srv__Trigger_Response__init(&dribble_res);

    // create launch service to trigger ball launch sequence  
    ESP_ERROR_CHECK(rclc_service_init_default(&service, node, support, service_name));
    ESP_ERROR_CHECK(rclc_executor_add_service(exec, &service, &req, &res, service_callback));

    // create launch service to trigger ball launch sequence  
    ESP_ERROR_CHECK(rclc_service_init_default(&dribble_service, node, support, dribble_service_name));
    ESP_ERROR_CHECK(rclc_executor_add_service(exec, &dribble_service, &dribble_req, &dribble_res, dribble_service_callback));
};


void ballHandler::declareParameters(){
    urosHandler::addParameter_bool("ballHandler_debug", &params.debug, &params);
    urosHandler::addParameter_int("FINGER_ON_TO_GRIPPER_OFF", &drbWorker.params.p_FINGER_ON_TO_GRIPPER_OFF, &drbWorker.params);
    urosHandler::addParameter_int("FINGER_RETRACT_WAIT_MS", &drbWorker.params.p_FINGER_RETRACT_WAIT_MS, &drbWorker.params);
    urosHandler::addParameter_int("GRAB_DELAY_MS", &drbWorker.params.p_GRAB_DELAY_MS, &drbWorker.params);
    urosHandler::addParameter_int("POST_DRIBBLE_MS", &drbWorker.params.p_POST_DRIBBLE_MS, &drbWorker.params);

};



void ballHandler::hw_task_callback(){

    while (1){
        // get gpio levels, active state is logic low
        current_state.armLimiterState[0] = ! gpio_get_level(cfg.armLimiterExterior);
        current_state.armLimiterState[1] = ! gpio_get_level(cfg.armLimiterInterior);
        current_state.gripper_pir_state = ! gpio_get_level(cfg.gripper_pir);

        if(current_state.armLimiterState[0]) cfg.decoder_handle->reset(cfg.arm);

        // update gpio levels if dribbleWorker is non functional
        if(!drbWorker.getRunningStatus()){
            gpio_set_level(cfg.finger, current_state.finger_state);
            gpio_set_level(cfg.gripper, current_state.gripper_state);
        }

        // calculate speeds from encoder ticks
        cfg.decoder_handle->update();
        current_state.feedbackFlyWheelSpeed_L = cfg.decoder_handle->count[cfg.flyWheelLower] - current_state.encoderFeedBack[cfg.flyWheelLower];
        current_state.feedbackFlyWheelSpeed_U = cfg.decoder_handle->count[cfg.flyWheelUpper] - current_state.encoderFeedBack[cfg.flyWheelUpper];

        // copy current encoder state
        memcpy(&current_state.encoderFeedBack, cfg.decoder_handle->count, DECODER_MAX_WHEEL_COUNT * sizeof(float));
        
        //  speed 
        cfg.qmd_handler->speeds[cfg.flyWheelLower] = current_state.flyWheelSpeed_L;
        cfg.qmd_handler->speeds[cfg.flyWheelUpper] = current_state.flyWheelSpeed_U;

        // check if internal limit switch is triggered, restrict angle to leaning down
        if(!(current_state.armLimiterState[1] && current_state.flywheel_angle < cfg.qmd_handler->speeds[cfg.flyWheelAngleRight]))
            cfg.qmd_handler->speeds[cfg.flyWheelAngleLeft] = cfg.qmd_handler->speeds[cfg.flyWheelAngleRight] = current_state.flywheel_angle;

        // if limit switch is triggered, do not move toward that direction 
        if( (current_state.armLimiterState[0] && current_state.arm_state < 0.0f) || 
            (current_state.armLimiterState[1] && current_state.arm_state > 0.0f)   ) 
            cfg.qmd_handler->speeds[cfg.arm] = 0.0f;
        else {
            cfg.qmd_handler->speeds[cfg.arm] = current_state.arm_state;
        }
            
        ESP_LOGI(TAG, "arm state %f arm enc %f", current_state.arm_state, cfg.decoder_handle->count[cfg.arm]);
        // ESP_LOGI(TAG, "flyWheelU %f flyWheelL %f", cfg.qmd_handler->speeds[cfg.flyWheelUpper], cfg.qmd_handler->speeds[cfg.flyWheelLower]);

        cfg.qmd_handler->update();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    vTaskDelete(NULL);
};

void ballHandler::flyWheel_subs_callback(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    // if(def) def->current_state.flyWheelSpeed = msg->data;

    ball_handler_state_t& state = def->dbgWorker.ctx.target;
    state.flyWheelSpeed = msg->data;
    def->eventInput(DEBUG_EVENT);

    ESP_LOGD(TAG, "flyWheel state : %f", msg->data);
}

void ballHandler::arm_subs_callback(const void* msgin){
    
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    ball_handler_state_t& state = def->dbgWorker.ctx.target;
    state.arm_state = msg->data;

    ESP_LOGD("armSub", "received arm state %f", state.arm_state);
    def->eventInput(DEBUG_EVENT);

}

void ballHandler::angle_subs_callback(const void* msgin){
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32*)msgin;

    // memcpy(msg->data, &mapped, sizeof(float));
    if(def && msg->data <= 1.0f && msg->data >= 0.0f) {
        ball_handler_state_t& state = def->dbgWorker.ctx.target;
        state.flywheel_angle = msg->data;
        def->eventInput(DEBUG_EVENT);
    };
}

void ballHandler::service_callback(const void * req, void *res)
{
    dhanush_srv__srv__SpeedAngle_Request * req_in = (dhanush_srv__srv__SpeedAngle_Request*)   req;
    dhanush_srv__srv__SpeedAngle_Response * res_in = (dhanush_srv__srv__SpeedAngle_Response*) res;

    ESP_LOGI(TAG, "%s service called with angle %lf speed %lf", service_name, req_in->angle, req_in->speed);    


    ball_handler_state_t& state = def->lnchWorker.ctx.target;
    state.launchState = ball_handler_state_t::LAUNCH_BEGIN;
    state.flywheel_angle = req_in->angle;
    state.flyWheelSpeed = req_in->speed;

    def->eventInput(THROW_SERVICE);
    res_in->success = true;
}


void ballHandler::dribble_service_callback(const void * req, void *res)
{
    std_srvs__srv__Trigger_Request * req_in = (std_srvs__srv__Trigger_Request*)   req;
    std_srvs__srv__Trigger_Response * res_in = (std_srvs__srv__Trigger_Response*) res;

    (void) req_in;

    ESP_LOGI(TAG, "%s service called ", service_name);    


    ball_handler_state_t& state = def->drbWorker.ctx.target;
    state.dribbleState = ball_handler_state_t::DRIBBLE_BEGIN;

    def->eventInput(DRIBBLE_SERVICE);
    res_in->success = true;
}


void ballHandler::finger_subs_callback(const void* msgin){
    
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

    ball_handler_state_t& state = def->dbgWorker.ctx.target;
    // state.finger_state =  ! msg->data;
    state.gripper_state = msg->data;


    def->eventInput(DEBUG_EVENT);
    ESP_LOGD(TAG, "finger state: %d", msg->data);
}



// Dubug worker accesses all the indexes of qmd
void debugWorker::run(){

    while (true){
        queryRunningState();

        // handle all the inputs from debug topic
        ctx.current->flyWheelSpeed_U = ctx.current->flyWheelSpeed_L  = 
            (1.0 - alpha) * ctx.current->flyWheelSpeed_L + (alpha)  * ctx.target.flyWheelSpeed;

        ctx.current->finger_state = ctx.target.finger_state;
        ctx.current->gripper_state = ctx.target.gripper_state;

        ctx.current->flywheel_angle = ctx.target.flywheel_angle;
        
        // TODO: PID implementation of arm state
        // ctx.current->arm_state
        ctx.current->arm_state = ctx.target.arm_state;

        ESP_LOGI(DBG_WRKR, "arm %f flywheel %f finger %d angle %f", 
            ctx.target.arm_state,
            ctx.target.flyWheelSpeed,
            ctx.target.finger_state,
            ctx.target.flywheel_angle
        );
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
};


// Dubug worker accesses all the indexes of qmd
void launchWorker::run(){

    while (true){
        queryRunningState();

        // keyframes of launch task
        switch (ctx.target.launchState)
        {
        case ball_handler_state_t::LAUNCH_BEGIN: {
            ctx.cfg->armController.position = ARM_REST_POS;
            ctx.cfg->flylController.speed = ctx.cfg->flyuController.speed = ctx.target.flyWheelSpeed;
            ctx.current->flywheel_angle = ctx.target.flywheel_angle;
            // ctx.current->flyWheelSpeed_L = ctx.current->flyWheelSpeed_U = ctx.target.flyWheelSpeed;
        }; break;


        case ball_handler_state_t::LAUNCH_PRE_THROW: {
            ctx.cfg->armController.position = ARM_IN_POS;
            ctx.cfg->flylController.speed = ctx.cfg->flyuController.speed = ctx.target.flyWheelSpeed;
            ctx.current->flywheel_angle = ctx.target.flywheel_angle;
            // ctx.current->flyWheelSpeed_L = ctx.current->flyWheelSpeed_U = ctx.target.flyWheelSpeed;

            ctx.current->gripper_state = false;
        }; break;

        case ball_handler_state_t::LAUNCH_POST_THROW: {
            ctx.cfg->armController.position = ARM_REST_POS;
            ctx.cfg->flylController.speed = ctx.cfg->flyuController.speed = 0.0f;
            ctx.current->flywheel_angle = ctx.target.flywheel_angle;
            // ctx.current->flyWheelSpeed_L = ctx.current->flyWheelSpeed_U = 0.0f;

        }; break;
        
        default:
            break;
        }

        // call update to pid controllers
        ctx.cfg->armController.update();
        ctx.cfg->flylController.update();
        ctx.cfg->flyuController.update();


        //check for state transitions 
        switch (ctx.target.launchState)
        {
        case ball_handler_state_t::LAUNCH_BEGIN: 
            if(ctx.cfg->armController.reached() && 
                ctx.cfg->flylController.reached() && 
                ctx.cfg->flyuController.reached()) {
                    ctx.target.launchState = ball_handler_state_t::LAUNCH_PRE_THROW;
                };

        break;


        case ball_handler_state_t::LAUNCH_PRE_THROW: {
            // if the target is reached or any limiter switch is hit
            if(ctx.cfg->armController.reached() || 
                ctx.current->armLimiterState[0] || 
                ctx.current->armLimiterState[1]) {
                    vTaskDelay(pdMS_TO_TICKS(THROW_WAIT_MS));
                    ctx.target.launchState = ball_handler_state_t::LAUNCH_POST_THROW;
                };

        }; 
        break;

        case ball_handler_state_t::LAUNCH_POST_THROW: {
            if(ctx.cfg->armController.reached() && 
                ctx.cfg->flylController.reached() && 
                ctx.cfg->flylController.reached()) ctx.target.launchState = ball_handler_state_t::LAUNCH_COMPLETE;
        }; 
        break;
        
        default:
            break;
        }

        ESP_LOGI(LNCH_WRKR, "current state %d", ctx.target.launchState);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
};





#define GRIPPER_ON 1
#define GRIPPER_OFF 0
#define FINGER_ON 1
#define FINGER_OFF 0

void dribbleWorker::run(){

    while (true){
        queryRunningState();

        // keyframes of launch task
        switch (ctx.target.dribbleState)
        {
        case ball_handler_state_t::DRIBBLE_BEGIN: {
            // TODO implement avtive / passive dribble switching
            // ctx.cfg->armController.position = ARM_REST_POS;
        }; break;


        case ball_handler_state_t::DRIBBLE_READY: {
            gpio_set_level(ctx.cfg->gripper, ctx.current->gripper_state = GRIPPER_ON);
            ctx.cfg->armController.position = ARM_OUT_POS;
        }; break;
        
        // time critical section
        case ball_handler_state_t::DRIBBLE_PRE_THROW: {

            gpio_set_level(ctx.cfg->finger, ctx.current->finger_state = FINGER_ON);    // finger on
            vTaskDelay(pdMS_TO_TICKS(params.p_FINGER_ON_TO_GRIPPER_OFF));              // wait
            gpio_set_level(ctx.cfg->gripper, ctx.current->gripper_state = GRIPPER_OFF);// gripper_off
            vTaskDelay(pdMS_TO_TICKS(params.p_FINGER_RETRACT_WAIT_MS));                // wait to retract finger
            gpio_set_level(ctx.cfg->finger, ctx.current->finger_state = FINGER_OFF);    // finger OFF
            
            // wait for ball to excees PIR range in forward trajectory 
            vTaskDelay(pdMS_TO_TICKS(params.p_POST_DRIBBLE_MS));
            
            // wait for pir to be triggered, enter forever wait
            while(gpio_get_level(ctx.cfg->gripper_pir)){
                vTaskDelay(pdMS_TO_TICKS(10));
            };
            
            gpio_set_level(ctx.cfg->gripper, ctx.current->gripper_state = GRIPPER_ON);

            ctx.target.dribbleState = ball_handler_state_t::DRIBBLE_POST_THROW;
        }; break;
            
        
        default:
            break;
        }

        
        // call update to pid controllers
        ctx.cfg->armController.update();


        //check for state transitions 
        switch (ctx.target.dribbleState)
        {
        case ball_handler_state_t::DRIBBLE_BEGIN: 
        // TODO implement active / passive dribble
            // if( ctx.cfg->armController.reached() ) ctx.target.dribbleState = ball_handler_state_t::DRIBBLE_READY;
            ctx.target.dribbleState = ball_handler_state_t::DRIBBLE_READY;
        break;


        case ball_handler_state_t::DRIBBLE_READY: {
            // if the target is reached or any limiter switch is hit
            if(ctx.cfg->armController.reached() || 
                ctx.current->armLimiterState[0] || 
                ctx.current->armLimiterState[1]) ctx.target.dribbleState = ball_handler_state_t::DRIBBLE_PRE_THROW;

        }; break;

        case ball_handler_state_t::DRIBBLE_POST_THROW: {
            if(ctx.cfg->armController.reached()) ctx.target.dribbleState = ball_handler_state_t::DRIBBLE_COMPLETE;
        }; break;
        
        default:
            break;
        }

        ESP_LOGI(DRIBL_WRKR, "current state %d", ctx.target.dribbleState);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
};


void ballHandler::eventInput(eventType type){

    switch (type)
    {
    case THROW_SERVICE:
        if(drbWorker.getRunningStatus()) drbWorker.block();
        if(dbgWorker.getRunningStatus()) dbgWorker.block();

        if(!lnchWorker.getRunningStatus()) lnchWorker.unblock();
        break;
        
        
    case DRIBBLE_SERVICE:
        if(lnchWorker.getRunningStatus()) lnchWorker.block();
        if(dbgWorker.getRunningStatus()) dbgWorker.block();

        if(!drbWorker.getRunningStatus()) drbWorker.unblock();
        break;
        
    case DEBUG_EVENT:
        if(drbWorker.getRunningStatus()) drbWorker.block();
        if(lnchWorker.getRunningStatus()) lnchWorker.block();

        if(!dbgWorker.getRunningStatus()) dbgWorker.unblock();
        break;

    default:
        break;
    }
};