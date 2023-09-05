#ifndef __ROBOT_RUNNER_H__
#define __ROBOT_RUNNER_H__

#include "../common/Utilities/PeriodicTask.h"
#include "RobotController.h"

class RobotRunner : public PeriodicTask
{
public:
    RobotRunner(RobotController *, PeriodicTaskManager *, float, std::string);

    using PeriodicTask::PeriodicTask;
    void init() override;
    void run() override;
    void cleanup() override;

    RobotController *_robot_ctrl;
};

#endif
