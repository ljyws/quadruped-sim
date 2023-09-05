#include <unistd.h>
#include "RobotRunner.h"


RobotRunner::RobotRunner(RobotController *robot_ctrl,PeriodicTaskManager *manager, float period,std::string name):PeriodicTask(manager,period,name)
{
    _robot_ctrl = robot_ctrl;
}