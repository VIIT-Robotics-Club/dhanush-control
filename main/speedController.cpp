

#include <speedController.hpp>


pid_config_t defSpeedController = {
};


// standard configuration for position controller 
pid_config_t defPositionController = {
    .p = 0.003f,
    .i = 0.00f,
    .d = 0.0001,
    .max = 1.0f,
    .min = -1.0f,
    .tolerance = 0.05,
    .iRange = 10,
};


speedController::speedController(pid_config_t& cfg) : pidController(cfg) {
    setpoint = &speed;
    process = &feedbackSpeed;
};


void speedController::setIo(qmd* qmd, decoder* dec, int index){
    decoderInput = &dec->count[index];
    output = &qmd->speeds[index];
    process = &feedbackSpeed;
};


void speedController::setIo(float* p_output, float* p_process){
    decoderInput = p_process;
    process = &feedbackSpeed;
    output = p_output;
};


float speedController::update(){
    feedbackSpeed = *decoderInput - prevTickCount;
    prevTickCount = *decoderInput;

    return pidController::update();
}; 



positionController::positionController(pid_config_t& p_cfg) : pidController(p_cfg) {
    setpoint = &position;
};

void positionController::setIo(qmd* qmd, decoder* dec, int index){
    output = &qmd->speeds[index];
    process = &dec->count[index];
    setpoint = &position;
};


void positionController::setIo(float* p_output, float* p_process){
    output = p_output;
    process = p_process;
    setpoint = &position;
};