#ifndef __CONTROL_FSM_H__
#define __CONTROL_FSM_H__

#include <iostream>
#include "FSM_State.h"
#include "ControlFSMData.h"
/*
 * 枚举所有操作模式
 */
enum class FSM_OperatingMode
{
    NORMAL,        // 正常运行
    TRANSITIONING, // 状态切换
    ESTOP,         // 停止状态
    EDAMP
};

struct FSM_StatesList
{
    FSM_State *invalid;
};

class ControlFSM
{

public:
    ControlFSM();

    void init();

    void runFSM();

    FSM_OperatingMode safetyPreCheck();

    //
    FSM_OperatingMode safetyPostCheck();

    FSM_State *getNextState(FSM_StateName stateName); // 获取下一个状态

    void printInfo();

    ControlFSMData data; // 包含所有控制数据

    FSM_StatesList statesList;   // FSM所有状态表
    FSM_State *currentState;     // 当前状态
    FSM_State *nextState;        // 下一状态
    FSM_StateName nextStateName; // 下一状态名

    // SafetyChecker<T> *safetyChecker;

    // TransitionData<T> transitionData;

    FSM_OperatingMode operatingMode; // FSM工作模式
    int printNum = 10000;            // N*(0.001s) in simulation time

    int printIter = 0; // make larger than printNum to not print

    int iter = 0;
};


#endif
