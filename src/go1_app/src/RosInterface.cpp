#include "RosInterface.h"
#include <stdio.h>

using namespace std;
Go1CtrlState aaa;

RosInterface::RosInterface(ros::NodeHandle &_nh)
{
    nh = _nh;

    // ROS publisher
    pub_joint_cmd[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FL_hip_controller/command", 1);
    pub_joint_cmd[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FL_thigh_controller/command", 1);
    pub_joint_cmd[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FL_calf_controller/command", 1);

    pub_joint_cmd[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FR_hip_controller/command", 1);
    pub_joint_cmd[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FR_thigh_controller/command", 1);
    pub_joint_cmd[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/FR_calf_controller/command", 1);

    pub_joint_cmd[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RL_hip_controller/command", 1);
    pub_joint_cmd[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RL_thigh_controller/command", 1);
    pub_joint_cmd[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RL_calf_controller/command", 1);

    pub_joint_cmd[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RR_hip_controller/command", 1);
    pub_joint_cmd[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RR_thigh_controller/command", 1);
    pub_joint_cmd[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("/go1_gazebo/RR_calf_controller/command", 1);

    sub_joint_msg[0] = nh.subscribe("/go1_gazebo/FL_hip_controller/state", 1, &RosInterface::FL_hip_state_callback, this);
    sub_joint_msg[1] = nh.subscribe("/go1_gazebo/FL_thigh_controller/state", 1, &RosInterface::FL_thigh_state_callback, this);
    sub_joint_msg[2] = nh.subscribe("/go1_gazebo/FL_calf_controller/state", 1, &RosInterface::FL_calf_state_callback, this);

    sub_joint_msg[3] = nh.subscribe("/go1_gazebo/FR_hip_controller/state", 1, &RosInterface::FR_hip_state_callback, this);
    sub_joint_msg[4] = nh.subscribe("/go1_gazebo/FR_thigh_controller/state", 1, &RosInterface::FR_thigh_state_callback, this);
    sub_joint_msg[5] = nh.subscribe("/go1_gazebo/FR_calf_controller/state", 1, &RosInterface::FR_calf_state_callback, this);

    sub_joint_msg[6] = nh.subscribe("/go1_gazebo/RL_hip_controller/state", 1, &RosInterface::RL_hip_state_callback, this);
    sub_joint_msg[7] = nh.subscribe("/go1_gazebo/RL_thigh_controller/state", 1, &RosInterface::RL_thigh_state_callback, this);
    sub_joint_msg[8] = nh.subscribe("/go1_gazebo/RL_calf_controller/state", 1, &RosInterface::RL_calf_state_callback, this);

    sub_joint_msg[9] = nh.subscribe("/go1_gazebo/RR_hip_controller/state", 1, &RosInterface::RR_hip_state_callback, this);
    sub_joint_msg[10] = nh.subscribe("/go1_gazebo/RR_thigh_controller/state", 1, &RosInterface::RR_thigh_state_callback, this);
    sub_joint_msg[11] = nh.subscribe("/go1_gazebo/RR_calf_controller/state", 1, &RosInterface::RR_calf_state_callback, this);

    sub_foot_contact_msg[0] = nh.subscribe("/visual/FL_foot_contact/the_force", 2, &RosInterface::FL_foot_contact_callback, this);
    sub_foot_contact_msg[1] = nh.subscribe("/visual/FR_foot_contact/the_force", 2, &RosInterface::FR_foot_contact_callback, this);
    sub_foot_contact_msg[2] = nh.subscribe("/visual/RL_foot_contact/the_force", 2, &RosInterface::RL_foot_contact_callback, this);
    sub_foot_contact_msg[3] = nh.subscribe("/visual/RR_foot_contact/the_force", 2, &RosInterface::RR_foot_contact_callback, this);

    go1_ctrl_state.reset();
}

bool RosInterface::send_cmd()
{
    unitree_legged_msgs::LowCmd low_cmd;
    for (int i = 0; i < 12; i++)
    {
        low_cmd.motorCmd[i].mode = 0x0A;
        low_cmd.motorCmd[i].q = 0;
        low_cmd.motorCmd[i].dq = 0;
        low_cmd.motorCmd[i].Kp = 0;
        low_cmd.motorCmd[i].Kd = 0;
        low_cmd.motorCmd[i].tau = 0.0; // go1_ctrl_state.joint_torques(i, 0);
        pub_joint_cmd[i].publish(low_cmd.motorCmd[i]);
    }

    return true;
}
void RosInterface::robot_init_stand()
{
    unitree_legged_msgs::LowCmd low_cmd;

    double init_pos[12] = {0.0, 0.77, -1.6, -0.0, 0.77, -1.6,
                           0.0, 0.77, -1.6, -0.0, 0.77, -1.6};

    for (int i = 0; i < 4; i++)
    {
        low_cmd.motorCmd[i * 3 + 0].mode = 0x0A;
        low_cmd.motorCmd[i * 3 + 0].Kp = 70;
        low_cmd.motorCmd[i * 3 + 0].dq = 0;
        low_cmd.motorCmd[i * 3 + 0].Kd = 3;
        low_cmd.motorCmd[i * 3 + 0].tau = 0;
        low_cmd.motorCmd[i * 3 + 1].mode = 0x0A;
        low_cmd.motorCmd[i * 3 + 1].Kp = 180;
        low_cmd.motorCmd[i * 3 + 1].dq = 0;
        low_cmd.motorCmd[i * 3 + 1].Kd = 8;
        low_cmd.motorCmd[i * 3 + 1].tau = 0;
        low_cmd.motorCmd[i * 3 + 2].mode = 0x0A;
        low_cmd.motorCmd[i * 3 + 2].Kp = 300;
        low_cmd.motorCmd[i * 3 + 2].dq = 0;
        low_cmd.motorCmd[i * 3 + 2].Kd = 15;
        low_cmd.motorCmd[i * 3 + 2].tau = 0;
    }

    for (int i = 0; i < 12; i++)
    {
        low_cmd.motorCmd[i].q = go1_ctrl_state.joint_pos[i];
    }

    double lastPos[12];
    double percent;
    for (int j = 0; j < 12; j++)
    {
        lastPos[j] = go1_ctrl_state.joint_pos[j];
    }

    for (int i = 1; i <= 1500; i++)
    {
        if (!ros::ok())
            break;
        percent = (double)i / 1500;
        cout<<percent<<endl;
        for (int j = 0; j < 12; j++)
        {
            low_cmd.motorCmd[j].q = lastPos[j] * (1 - percent) + init_pos[j] * percent;
            pub_joint_cmd[j].publish(low_cmd.motorCmd[j]);
        }
        ros::spinOnce();
        usleep(1000);
    }
}

// FL
void RosInterface::FL_hip_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state)
{
    // cout<<"imhere!"<<endl;
    go1_ctrl_state.joint_pos[0] = go1_joint_state.q;
    go1_ctrl_state.joint_vel[0] = go1_joint_state.dq;
    aaa.joint_pos[0] = go1_joint_state.q;
    //  cout<<aaa.joint_pos<<endl;
}

void RosInterface::FL_thigh_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state)
{
    go1_ctrl_state.joint_pos[1] = go1_joint_state.q;
    go1_ctrl_state.joint_vel[1] = go1_joint_state.dq;
    aaa.joint_pos[1] = go1_joint_state.q;
}

void RosInterface::FL_calf_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state)
{
    go1_ctrl_state.joint_pos[2] = go1_joint_state.q;
    go1_ctrl_state.joint_vel[2] = go1_joint_state.dq;
    aaa.joint_pos[2] = go1_joint_state.q;
}

// FR
void RosInterface::FR_hip_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state)
{
    go1_ctrl_state.joint_pos[3] = go1_joint_state.q;
    go1_ctrl_state.joint_vel[3] = go1_joint_state.dq;
    aaa.joint_pos[3] = go1_joint_state.q;
}

void RosInterface::FR_thigh_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state)
{
    go1_ctrl_state.joint_pos[4] = go1_joint_state.q;
    go1_ctrl_state.joint_vel[4] = go1_joint_state.dq;
    aaa.joint_pos[4] = go1_joint_state.q;
}

void RosInterface::FR_calf_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state)
{
    go1_ctrl_state.joint_pos[5] = go1_joint_state.q;
    go1_ctrl_state.joint_vel[5] = go1_joint_state.dq;
    aaa.joint_pos[5] = go1_joint_state.q;
}

// RL
void RosInterface::RL_hip_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state)
{
    go1_ctrl_state.joint_pos[6] = go1_joint_state.q;
    go1_ctrl_state.joint_vel[6] = go1_joint_state.dq;
    aaa.joint_pos[6] = go1_joint_state.q;
}

void RosInterface::RL_thigh_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state)
{
    go1_ctrl_state.joint_pos[7] = go1_joint_state.q;
    go1_ctrl_state.joint_vel[7] = go1_joint_state.dq;
    aaa.joint_pos[7] = go1_joint_state.q;
}

void RosInterface::RL_calf_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state)
{
    go1_ctrl_state.joint_pos[8] = go1_joint_state.q;
    go1_ctrl_state.joint_vel[8] = go1_joint_state.dq;
    aaa.joint_pos[8] = go1_joint_state.q;
}

// RR
void RosInterface::RR_hip_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state)
{
    go1_ctrl_state.joint_pos[9] = go1_joint_state.q;
    go1_ctrl_state.joint_vel[9] = go1_joint_state.dq;
    aaa.joint_pos[9] = go1_joint_state.q;
}

void RosInterface::RR_thigh_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state)
{
    go1_ctrl_state.joint_pos[10] = go1_joint_state.q;
    go1_ctrl_state.joint_vel[10] = go1_joint_state.dq;
    aaa.joint_pos[10] = go1_joint_state.q;
}

void RosInterface::RR_calf_state_callback(const unitree_legged_msgs::MotorState &go1_joint_state)
{
    go1_ctrl_state.joint_pos[11] = go1_joint_state.q;
    go1_ctrl_state.joint_vel[11] = go1_joint_state.dq;
    aaa.joint_pos[11] = go1_joint_state.q;
}

// foot contact force
void RosInterface::FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force)
{
    go1_ctrl_state.foot_force[0] = force.wrench.force.z;
}

void RosInterface::FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force)
{
    go1_ctrl_state.foot_force[1] = force.wrench.force.z;
}

void RosInterface::RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force)
{
    go1_ctrl_state.foot_force[2] = force.wrench.force.z;
}

void RosInterface::RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force)
{
    go1_ctrl_state.foot_force[3] = force.wrench.force.z;
}
