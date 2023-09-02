#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include "RosInterface.h"

extern Go1CtrlState aaa;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go1_ctrl");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    std::string use_sim_time;
    if (ros::param::get("/use_sim_time", use_sim_time))
    {
        if (use_sim_time != "true")
        {
            std::cout << "ROS must set use_sim_time in order to use this program!" << std::endl;
            return -1;
        }
    }

    ros::NodeHandle nh;
    std::unique_ptr<RosInterface> go1 = std::make_unique<RosInterface>(nh);

    ros::Duration(3).sleep();

    ros::spinOnce();

    go1->robot_init_stand();
    // go1->log_all_q();

    while (ros::ok())
    {
        // std::cout << aaa.joint_pos << std::endl;
        ros::spinOnce();
        usleep(1000);
    }
    return 0;
}