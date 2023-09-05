#include <cassert>
#include <iostream>

#include "main_app.h"
#include "RobotController.h"
#include "../robot/SimulationBridge.h"
MasterConfig gMasterConfig;

int main_app(int argc, char **argv, RobotController *ctrl)
{
    gMasterConfig._robot = RobotType::GO1; // 初始化机器人为Go1

    gMasterConfig.simulated = true; // 选择为仿真环境

    gMasterConfig.load_from_file = false; // 不从外部文件导入配置

    printf("[----- Quadruped -----]\n");
    printf("Robot:   %s\n", gMasterConfig._robot == RobotType::GO1 ? "Go1" : "A1");
    printf("RunMode: %s\n", gMasterConfig.simulated ? "Simulation": "Quadruped");
    if (gMasterConfig.simulated) // 仿真环境下
    {
        SimulationBridge simulationBridge(gMasterConfig._robot, ctrl);
        // simulationBridge.run();
    }
}
