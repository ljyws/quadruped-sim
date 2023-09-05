#ifndef __SIMULATION_BRIDGE_H__
#define __SIMULATION_BRIDGE_H__

#include "../common/cppTypes.h"
#include "RobotController.h"
#include "../common/Utilities/PeriodicTask.h"
#include "../common/ControlParameters/RobotParameters.h"
#include "RobotRunner.h"
class SimulationBridge
{

public:
    explicit SimulationBridge(RobotType robot, RobotController *robot_ctrl) : _robot(robot)
    {
        // _fakeTaskManager = new PeriodicTaskManager;
        // _robotRunner = new RobotRunner(robot_ctrl,_fakeTaskManager,0,"robot-task");
    }

    void run();
    void handleControlParameters();
    void runRobotControl();
    ~SimulationBridge(){ }

private:
    PeriodicTaskManager taskManager;
    bool _firstControllerRun = true;
    PeriodicTaskManager *_fakeTaskManager = nullptr;
    RobotType _robot;
    RobotRunner *_robotRunner = nullptr;
    RobotControlParameters _robotParameters;
    ControlParameters *_userParameters;
};

#endif
