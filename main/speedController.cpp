

#include <speedController.hpp>



const float smoothSpeedController::SMOOTHER_OFF_PERCENT = 0.01f;

pid_config_t defSpeedController = {
    .a = 1.0f
};


// standard configuration for position controller 
pid_config_t defPositionController = {
    .p = 0.003f,
    .i = 0.00f,
    .d = 0.0001,
    .max = 0.4f,
    .min = -0.4f,
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

    float out =  a * speed;
    // *output = out;
    return out;
}; 



smoothSpeedController::smoothSpeedController(pid_config_t& cfg, float coeff) : speedController(cfg), coefficient(coeff) {};

float smoothSpeedController::update(){
    // get response from pid controller and filter it with given coefficient
    targetOutput = speedController::update();
    smoothedOutput = (1.0 - coefficient) * smoothedOutput + coefficient * targetOutput;

    *output = smoothedOutput;
    return smoothedOutput;
};


bool smoothSpeedController::reached(){
    return speedController::reached() && abs((targetOutput - smoothedOutput) / targetOutput) < SMOOTHER_OFF_PERCENT;
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