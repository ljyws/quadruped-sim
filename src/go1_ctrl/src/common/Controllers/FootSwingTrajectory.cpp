#include "FootSwingTrajectory.h"
#include "../Math/Interpolation.h"

void FootSwingTrajectory::computeSwingTrajectoryBezier(float phase, float swingTime)
{

    _p = Interpolate::cubicBezier<Vec3<float>>(_p0, _pf, phase);
    _v = Interpolate::cubicBezierFirstDerivative<Vec3<float>>(_p0, _pf, phase) / swingTime;
    _a = Interpolate::cubicBezierSecondDerivative<Vec3<float>>(_p0, _pf, phase) / (swingTime * swingTime);

    float zp, zv, za; 
    if (phase < (0.5))
    {
        zp = Interpolate::cubicBezier<float>(_p0[2], _p0[2] + _height, phase * 2);
        zv = Interpolate::cubicBezierFirstDerivative<float>(_p0[2], _p0[2] + _height, phase * 2) * 2 / swingTime;
        za = Interpolate::cubicBezierSecondDerivative<float>(_p0[2], _p0[2] + _height, phase * 2) * 4 / (swingTime * swingTime);
    }
    else
    {
        zp = Interpolate::cubicBezier<float>(_p0[2] + _height, _pf[2], phase * 2 - 1);
        zv = Interpolate::cubicBezierFirstDerivative<float>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 2 / swingTime;
        za = Interpolate::cubicBezierSecondDerivative<float>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
    }

    _p[2] = zp;
    _v[2] = zv;
    _a[2] = za;
}
