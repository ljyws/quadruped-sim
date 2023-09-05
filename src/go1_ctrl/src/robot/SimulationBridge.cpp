#include "SimulationBridge.h"

void SimulationBridge::run()
{
    runRobotControl();
}

void SimulationBridge::runRobotControl()
{
    if (_firstControllerRun)
    {
        printf("[Simulator Driver] First run of robot controller...\n");


        auto *userControlParameters = _robotRunner->_robot_ctrl->getUserControlParameters();
        if(userControlParameters)
        {

        }
        else
        {
        
        }
        _robotRunner->init();
        _firstControllerRun = false;
    }
    _robotRunner->run();
}