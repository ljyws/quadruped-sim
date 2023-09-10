#include "SimulationBridge.h"

void SimulationBridge::run()
{
    printf("[--------------- Starting Main Loop ----------------]\n");
    for (;;)
    {
        runRobotControl();
    }

    // runRobotControl();
}


void SimulationBridge::handleControlParameters()
{
    
}



void SimulationBridge::runRobotControl()
{
    if (_firstControllerRun)
    {
        printf("First run of robot controller!\n");
        if (_robotParameters.isFullyInitialized())
        {
            printf("\tAll %ld control parameters are initialized\n",_robotParameters.collection._map.size());
        }
        auto *userControlParameters = _robotRunner->_robot_ctrl->getUserControlParameters();
    
        // if(userControlParameters)
        // {

        // }
        // else
        // {

        // }
        // _robotRunner->init();
        _firstControllerRun = false;
    }
    // _robotRunner->run();
}