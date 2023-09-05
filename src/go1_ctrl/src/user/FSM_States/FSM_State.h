#ifndef __FSM_STATE_H__
#define __FSM_STATE_H__

#include "Eigen/Dense"

enum class FSM_StateName
{
  INVALID,
  PASSIVE,
  JOINT_PD,
  IMPEDANCE_CONTROL,
  STAND_UP,
  BALANCE_STAND,
  LOCOMOTION,
  RECOVERY_STAND,
  VISION,
  BACKFLIP,
  FRONTJUMP
};

class FSM_State
{

public:
  

  FSM_State();
};

#endif
