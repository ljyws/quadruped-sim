#ifndef __ROS_INTERFACE_H__
#define __ROS_INTERFACE_H__

#include <Eigen/Dense>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/LowCmd.h>


#include "Go1Params.h"
#include "Go1CtrlState.h"
class RosInterface
{
public:
    RosInterface(ros::NodeHandle &_nh);

    bool send_cmd(void);

    void robot_init_stand(void);

    void log_all_q(void)
    {
        std::cout << go1_ctrl_state.joint_pos<<std::endl;
    }

    void FL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

    void FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

    void RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

    void RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);



private:
    ros::NodeHandle nh;

    // 0,  1,  2: FL_hip, FL_thigh, FL_calf
    // 3,  4,  5: FR_hip, FR_thigh, FR_calf
    // 6,  7,  8: RL_hip, RL_thigh, RL_calf
    // 9, 10, 11: RR_hip, RR_thigh, RR_calf
    ros::Subscriber sub_joint_msg[12];
    ros::Publisher pub_joint_cmd[12];

    ros::Publisher pub_euler_d;

    // 0, 1, 2, 3: FL, FR, RL, RR
    ros::Subscriber sub_foot_contact_msg[4];

    Go1CtrlState go1_ctrl_state;
    
};
#endif //__ROS_INTERFACE_H__
