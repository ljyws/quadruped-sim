#ifndef __GAIT_SCHEDULER_H__
#define __GAIT_SCHEDULER_H__

#include <iostream>
#include "../cppTypes.h"
#include "../../user/unitree_UserParameters.h"
// 枚举步态类型
enum class GaitType
{
    STAND,
    TROT,
    TRANSITION_TO_STAND
};


struct GaitData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 字节对齐
    GaitData()
    {
        reset();
    }
    void reset();

    GaitType _currentGait; // 当前步态

    GaitType _nextGait; // 下一步态

    std::string gaitName; // 步态名字

    float periodTImeNominal;
    float periodTimeNominal;     // 总周期时间
    float initialPhase;          // 初始偏移相位
    float switchingPhaseNominal; // 相位切换点
    int overrideable;        // 允许步态参数被覆写

    Eigen::Vector4i gaitEnabled;

    // 基于时间描述符
    Vec4<float> periodTime;          //  整个步态时间
    Vec4<float> timeStance;          //  总站立时间
    Vec4<float> timeSwing;           // 总摆动时间
    Vec4<float> timeStanceRemaining; //  剩余站立时间
    Vec4<float> timeSwingRemaining;  //  剩余摆动时间

    Vec4<float> switchingPhase; // phase to switch to swing 相切换到swing
    Vec4<float> phaseVariable;  // overall gait phase for each foot 每只脚的整体步态阶段
    Vec4<float> phaseOffset;    // nominal gait phase offsets 步态相位偏移
    Vec4<float> phaseScale;     // phase scale relative to variable 相对于变量的相位刻度
    Vec4<float> phaseStance;    // stance subphase 支撑相中相位
    Vec4<float> phaseSwing;     // swing subphase 摆动相中相位

    // Scheduled contact states 预定的接触状态
    Eigen::Vector4i contactStateScheduled; //  脚的接触状态
    Eigen::Vector4i contactStatePrev;      //  脚的先前接触状态
    Eigen::Vector4i touchdownScheduled;    //  预定的触地事件标志
    Eigen::Vector4i liftoffScheduled;      //  预定离地事件标值
};

class GaitScheduler
{
public:
    GaitScheduler(unitree_UserParameters*_userParameters, float _dt);
    ~GaitScheduler(){};

    void init();

    void loop();

    void modifyGait();

    void createGait();

    GaitData gaitData;
    //周期时间
    float period_time_natural = 0.5;
    // 摆动向接触切换点
    float switching_phase_natural = 0.5;
    // 摆动时间
    float swing_time_natural = 0.25;


private:
    float dt; //timestep

    float dphase;

};

#endif
