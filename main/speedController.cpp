

#include <speedController.hpp>


speedController::speedController(qmd* qmd, decoder* dec, int index) {
    decoderInput = &dec->count[index];
    output = &qmd->speeds[index];
    process = &feedback;
    setpoint = &speed;
};


float speedController::update(){
    feedback = *decoderInput - prevTickCount;
    prevTickCount = *decoderInput;

    return pidController::update();
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