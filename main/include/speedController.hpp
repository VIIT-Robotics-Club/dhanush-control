
#ifndef SPEED_CONTROLLER_HPP
#define SPEED_CONTROLLER_HPP


#include <pidController.hpp>
#include <qmd.hpp>
#include <quadrature.hpp>


class speedController : public pidController {
public:
    speedController(qmd* qmd_handler, decoder* dec, int index = 0);

    float speed = 0.0f, error_tolerance = 0.0f;

    float update();

    inline bool reached() { return abs(signals[0]) <= error_tolerance ;};

private:
    float feedback = 0.0f, prevTickCount = 0.0f;
    float* decoderInput;
};



class positionController : public pidController {
public:

    void setIo(qmd* qmd_handler, decoder* dec, int index = 0);
    void setIo(float* out, float* process);


    float position = 0., error_tolerance = 0.0f;
    
    inline bool reached() { return abs(signals[0]) <= error_tolerance ;};

};


#endif //  SPEED_CONTROLLER_HPP