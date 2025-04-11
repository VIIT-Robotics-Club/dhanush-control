

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


positionController::positionController(qmd* qmd, decoder* dec, int index){
    output = &qmd->speeds[index];
    process = &dec->count[index];
    setpoint = &position;
};

