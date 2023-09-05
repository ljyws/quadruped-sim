#include "GaitScheduler.h"

/*
 * 清空数据 初始化
 */
void GaitData::reset()
{

    _nextGait = _currentGait; // 停止一切步态切换

    periodTimeNominal = 0.0;     // 一个周期时间
    initialPhase = 0.0;          // 初始相位偏移
    switchingPhaseNominal = 0.0; // nominal phase to switch contacts
    overrideable = 0;            // allows the gait parameters to be overridden

    gaitEnabled = Eigen::Vector4i::Zero(); // enable gait controlled legs

    periodTime = Vec4<float>::Zero();          // overall gait period time
    timeStance = Vec4<float>::Zero();          // total stance time
    timeSwing = Vec4<float>::Zero();           // total swing time
    timeStanceRemaining = Vec4<float>::Zero(); // stance time remaining
    timeSwingRemaining = Vec4<float>::Zero();  // swing time remaining

    switchingPhase = Vec4<float>::Zero(); // phase to switch to swing
    phaseVariable = Vec4<float>::Zero();  // overall gait phase for each foot
    phaseOffset = Vec4<float>::Zero();    // nominal gait phase offsets
    phaseScale = Vec4<float>::Zero();     // phase scale relative to variable
    phaseStance = Vec4<float>::Zero();    // stance subphase
    phaseSwing = Vec4<float>::Zero();     // swing subphase

    contactStateScheduled = Eigen::Vector4i::Zero(); // contact state of the foot
    contactStatePrev = Eigen::Vector4i::Zero();      // previous contact state of the foot
    touchdownScheduled = Eigen::Vector4i::Zero();    // scheduled touchdown flag
    liftoffScheduled = Eigen::Vector4i::Zero();      // scheduled liftoff flag
}

GaitScheduler::GaitScheduler(float _dt)
{
    init();
    dt = _dt;
}

void GaitScheduler::init()
{
    std::cout << "[GAIT] Initialize Gait Scheduler" << std::endl;

    gaitData._currentGait = GaitType::STAND;

    gaitData.reset();

    createGait();

    period_time_natural = gaitData.periodTimeNominal;

    switching_phase_natural = gaitData.switchingPhaseNominal;
}

void GaitScheduler::loop()
{

    // Modify the gait with settings 修改步态设置（切换步态）
    modifyGait();
    
    // 非站立
    if (gaitData._currentGait != GaitType::STAND)
    {
        // Track the reference phase variable 跟踪参考相位变量
        gaitData.initialPhase = fmod((gaitData.initialPhase + (dt / gaitData.periodTimeNominal)), 1);
    }

    // Iterate over the feet 遍历四足
    for (int foot = 0; foot < 4; foot++)
    {
        // Set the previous contact state for the next timestep 设置前一个接触状态
        gaitData.contactStatePrev(foot) = gaitData.contactStateScheduled(foot);

        if (gaitData.gaitEnabled(foot) == 1)
        {
            // Monotonic time based phase incrementation 单调时基相位增量
            if (gaitData._currentGait == GaitType::STAND)
            {
                // Don't increment the phase when in stand mode 在站立模式下，不要增加相位
                dphase = 0.0;
            }
            else
            {
                dphase = gaitData.phaseScale(foot) * (dt / gaitData.periodTimeNominal); // 每次循环相位增量
            }

            // Find each foot's current phase 找到脚的当前相位 通过增加相位增量 并取余数来实现0~1
            gaitData.phaseVariable(foot) =
                fmod((gaitData.phaseVariable(foot) + dphase), 1);

            // Check the current contact state 检测当前接触状态
            if (gaitData.phaseVariable(foot) <= gaitData.switchingPhase(foot)) // 当前为接触状态
            {
                // Foot is scheduled to be in contact  足部预定进行接触
                gaitData.contactStateScheduled(foot) = 1;

                // Stance subphase calculation  在支撑相中的相位
                gaitData.phaseStance(foot) =
                    gaitData.phaseVariable(foot) / gaitData.switchingPhase(foot);

                // Swing phase has not started since foot is in stance  摇摆阶段还没有开始，因为脚在支撑
                gaitData.phaseSwing(foot) = 0.0;

                // Calculate the remaining time in stance 计算剩余支撑时间
                gaitData.timeStanceRemaining(foot) =
                    gaitData.periodTime(foot) *
                    (gaitData.switchingPhase(foot) - gaitData.phaseVariable(foot));

                // Foot is in stance, no swing time remaining 剩余摆动时间 当前腿在支撑 没有剩余摆动时间
                gaitData.timeSwingRemaining(foot) = 0.0;

                // First contact signifies scheduled touchdown  第一次接触表示预定着陆
                if (gaitData.contactStatePrev(foot) == 0) // 之前为摆动
                {
                    // Set the touchdown flag to 1 设置着陆标志为1
                    gaitData.touchdownScheduled(foot) = 1;
                }
                else
                {
                    // Set the touchdown flag to 0 设置着陆标志为0
                    gaitData.touchdownScheduled(foot) = 0;
                }
            }
            else
            {
                // Foot is not scheduled to be in contact 足端没有预定进行接触
                gaitData.contactStateScheduled(foot) = 0;

                // Stance phase has completed since foot is in swing  支撑相结束 在支撑相中的相位
                gaitData.phaseStance(foot) = 1.0;

                // Swing subphase calculation  Swing相中的相位计算
                gaitData.phaseSwing(foot) =
                    (gaitData.phaseVariable(foot) - gaitData.switchingPhase(foot)) /
                    (1.0 - gaitData.switchingPhase(foot));

                // Foot is in swing, no stance time remaining 摆动没有支撑时间剩余
                gaitData.timeStanceRemaining(foot) = 0.0;

                // Calculate the remaining time in swing 计算剩余摆动时间
                gaitData.timeSwingRemaining(foot) =
                    gaitData.periodTime(foot) * (1 - gaitData.phaseVariable(foot));

                // First contact signifies scheduled touchdown  第一次接触表示预定着陆
                if (gaitData.contactStatePrev(foot) == 1) // 之前为接触
                {
                    // Set the liftoff flag to 1 设置离地标志为1
                    gaitData.liftoffScheduled(foot) = 1;
                }
                else
                {
                    // Set the liftoff flag to 0 设置离地标志为1
                    gaitData.liftoffScheduled(foot) = 0;
                }
            }
        }
        else
        {
            // Leg is not enabled 腿未启用
            gaitData.phaseVariable(foot) = 0.0;

            // Foot is not scheduled to be in contact 脚没有接触计划
            gaitData.contactStateScheduled(foot) = 0;
        }
    }
}

void GaitScheduler::modifyGait()
{
    if (gaitData._currentGait != gaitData._nextGait)
    {
        createGait();
    }
}

/**
 * 创建标准步态，你只需要定义如下参数
 *
 *   gaitData.periodTimeNominal
 *   gaitData.switchingPhaseNominal
 *   gaitData.phaseOffset
 * 其余的可以设置为：
 *
 *   gaitData.gaitEnabled << 1, 1, 1, 1;
 *   gaitData.initialPhase = 0.0;
 *   gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
 *
 * These add flexibility to be used for very irregular gaits and transitions.
 * 这些增加了灵活性，以用于非常不规则的步态和过渡
 */
void GaitScheduler::createGait()
{
    std::cout << "[GAIT] Transitioning gait from " << gaitData.gaitName << " to ";

    switch (gaitData._nextGait)
    {
    case GaitType::STAND:
        gaitData.gaitName = "STAND";
        gaitData.gaitEnabled << 1, 1, 1, 1;
        gaitData.periodTimeNominal = 10.0;
        gaitData.initialPhase = 0.0;
        gaitData.switchingPhaseNominal = 1.0;
        gaitData.phaseOffset << 0.5, 0.5, 0.5, 0.5;
        gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
        gaitData.overrideable = 0;
        break;

    case GaitType::TROT:
        gaitData.gaitName = "TROT";
        gaitData.gaitEnabled << 1, 1, 1, 1;
        gaitData.periodTimeNominal = 0.5;
        gaitData.initialPhase = 0.0;
        gaitData.switchingPhaseNominal = 0.5;
        gaitData.phaseOffset << 0.0, 0.5, 0.5, 0.0;
        gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
        gaitData.overrideable = 1;
        break;
    default:
        break;
    }

    gaitData._currentGait = gaitData._nextGait;

    std::cout << gaitData.gaitName << "\n"
              << std::endl;
}
