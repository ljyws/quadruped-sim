#ifndef __ROBOT_CONTROLLER_H__
#define __ROBOT_CONTROLLER_H__

#include "../common/cppTypes.h"
#include "../common/Dynamics/Quadruped.h"
#include "../common/ControlParameters/RobotParameters.h"
class RobotController
{
    friend class RobotRunner;

public:
    RobotController(){}
    virtual ~RobotController(){}

    virtual void initializeController() = 0;

    virtual void runController() = 0;

    virtual ControlParameters *getUserControlParameters() = 0;

protected:
    Quadruped *_quadruped = nullptr;
    RobotControlParameters *_controlParameters = nullptr;
    RobotType _robotType;
};

#endif