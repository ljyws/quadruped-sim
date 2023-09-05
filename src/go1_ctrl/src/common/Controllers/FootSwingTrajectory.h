#ifndef __FOOTSWINGTRAJECTORY_H__
#define __FOOTSWINGTRAJECTORY_H__

#include "../cppTypes.h"

class FootSwingTrajectory
{
public:
    FootSwingTrajectory()
    {
        _p0.setZero(); // 初始点
        _pf.setZero(); // 终点
        _p.setZero();  // 轨迹点
        _v.setZero();  // 轨迹速度
        _a.setZero();  // 轨迹加速度
        _height = 0;   // 轨迹高度
    }

    /*!
     * 设置脚的起始位置
     * @param p0 : the initial foot position
     */
    void setInitialPosition(Vec3<float> p0)
    {
        _p0 = p0;
    }

    /*!
     * 设置脚的终点位置
     * @param pf : the final foot posiiton
     */
    void setFinalPosition(Vec3<float> pf)
    {
        _pf = pf;
    }

    /*!
     * 最大高度
     * @param h : the maximum height of the swing, achieved halfway through the swing 在一半路程到达最高
     */
    void setHeight(float h)
    {
        _height = h;
    }

    void computeSwingTrajectoryBezier(float phase, float swingTime);

    /*!
     * Get the foot position at the current point along the swing//获得轨迹坐标
     * @return : the foot position
     */
    Vec3<float> getPosition()
    {
        return _p;
    }

    /*!
     * Get the foot velocity at the current point along the swing//获得此时轨迹导数
     * @return : the foot velocity
     */
    Vec3<float> getVelocity()
    {
        return _v;
    }

    /*!
     * Get the foot acceleration at the current point along the swing 得到脚在当前点上的加速
     * @return : the foot acceleration
     */
    Vec3<float> getAcceleration()
    {
        return _a;
    }

private:
    Vec3<float> _p0, _pf, _p, _v, _a;
    float _height;
};

#endif
