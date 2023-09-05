#ifndef __UNITREE_CONTROLLER_H__
#define __UNITREE_CONTROLLER_H__

#include "../robot/RobotController.h"
#include "../common/ControlParameters/ControlParameters.h"
#include "unitree_UserParameters.h"
#include "FSM_States/ControlFSM.h"
#include "../common/Controllers/GaitScheduler.h"
class unitree_Controller : public RobotController
{
public:
    unitree_Controller();
    virtual ~unitree_Controller(){}

    virtual void initializeController();
    virtual void runController();

    virtual ControlParameters *getUserControlParameters()
    {
        return &userParameters;
    }

protected:

    ControlFSM *_controlFSM;

    GaitScheduler *_gaitScheduler;

    unitree_UserParameters userParameters;
};

#endif
