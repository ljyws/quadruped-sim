#ifndef __GO1_CONTROLLER_H__
#define __GO1_CONTROLLER_H__


#include <iostream>
#include <string>

#include <ros/ros.h>
#include "RosInterface.h"

class Go1Controller
{
public:
    Go1Controller(ros::NodeHandle &_nh);

    void robot_init_stand();


};


#endif

