
#ifndef SPEED_CONTROLLER_HPP
#define SPEED_CONTROLLER_HPP


#include <pidController.hpp>
#include <qmd.hpp>
#include <quadrature.hpp>


#define SMOOTHER_COEFFICIENT 0.1f


// standard configuration for position controller 
extern pid_config_t defSpeedController;
extern pid_config_t defPositionController;



class speedController : public pidController {
public:
    float speed = 0.0f, error_tolerance = 0.0f;

    speedController(pid_config_t& p_cfg = defSpeedController);


    void setIo(qmd* qmd_handler, decoder* dec, int index = 0);
    void setIo(float* out, float* process);


    float update();

    // TODO impl pid checking on tested implementation
    inline bool reached() { return true ;};

protected:
    float feedbackSpeed = 0.0f, prevTickCount = 0.0f;
    float* decoderInput;
};


class smoothSpeedController : public speedController{
public:
    const static float SMOOTHER_OFF_PERCENT;

    smoothSpeedController(pid_config_t& p_cfg = defSpeedController, float coefficient = SMOOTHER_COEFFICIENT);

    float update();

    bool reached();

private:
    float coefficient = SMOOTHER_COEFFICIENT; 
    float smoothedOutput = 0.0f, targetOutput = 0.0f;
};



class positionController : public pidController {
public:

    positionController(pid_config_t& p_cfg = defPositionController);

    void setIo(qmd* qmd_handler, decoder* dec, int index = 0);
    void setIo(float* out, float* process);


    float position = 0.0f, error_tolerance = 0.0f;
    
    inline bool reached() { return abs(error) <= error_tolerance ;};

};


#endif //  SPEED_CONTROLLER_HPP