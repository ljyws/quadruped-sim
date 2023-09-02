#include <Eigen/Dense>
#include <ros/ros.h>
#include "Go1Params.h"

class Go1CtrlState
{
public:
    Go1CtrlState()
    {
        reset();
    }

    void reset()
    {
        gait_type = 1;
        gait_type_last = 1;

        root_pos_d.setZero();
        root_euler_d.setZero();
        root_lin_vel_d.setZero();
        root_ang_vel_d.setZero();

        robot_mass = 12.84;

        go1_trunk_inertia << 0.0168128557, 0.0, 0.0,
            0.0, 0.063009565, 0.0,
            0.0, 0.0, 0.0716547275;

        default_foot_pos << 0.17, 0.17, -0.17, -0.17,
            0.15, -0.15, 0.15, -0.15,
            -0.35, -0.35, -0.35, -0.35;

        root_pos.setZero();
        root_quat.setIdentity();
        root_euler.setZero();
        root_rot_mat.setZero();
        root_rot_mat_z.setZero();
        root_lin_vel.setZero();
        root_ang_vel.setZero();
        root_acc.setZero();

        foot_force.setZero();

        joint_pos.setZero();
        joint_vel.setZero();

        foot_pos_target_world.setZero();
        foot_pos_target_abs.setZero();
        foot_pos_target_rel.setZero();
        foot_pos_start.setZero();
        foot_pos_world.setZero();
        foot_pos_abs.setZero();
        foot_pos_rel.setZero();
        foot_pos_abs_mpc.setZero();
        foot_pos_rel_last_time.setZero();
        foot_pos_target_last_time.setZero();
        foot_pos_cur.setZero();
        foot_pos_recent_contact.setZero();
        foot_vel_world.setZero();
        foot_vel_abs.setZero();
        foot_vel_rel.setZero();
        j_foot.setIdentity();

        for (int i = 0; i < NUM_LEG; ++i)
        {
            contacts[i] = false;
            plan_contacts[i] = false;
            early_contacts[i] = false;
        }
        gait_counter_speed << 2, 2, 2, 2;

        double kp_foot_x = 300.0;
        double kp_foot_y = 400.0;
        double kp_foot_z = 400.0;

        double kd_foot_x = 8.0;
        double kd_foot_y = 8.0;
        double kd_foot_z = 8.0;

        kp_foot << kp_foot_x, kp_foot_x, kp_foot_x, kp_foot_x,
            kp_foot_y, kp_foot_y, kp_foot_y, kp_foot_y,
            kp_foot_z, kp_foot_z, kp_foot_z, kp_foot_z;
        kd_foot << kd_foot_x, kd_foot_x, kd_foot_x, kd_foot_x,
            kd_foot_y, kd_foot_y, kd_foot_y, kd_foot_y,
            kd_foot_z, kd_foot_z, kd_foot_z, kd_foot_z;

        km_foot = Eigen::Vector3d(0.1, 0.1, 0.1);

        kp_linear = Eigen::Vector3d(1000.0, 1000.0, 1000.0);
        kd_linear = Eigen::Vector3d(200.0, 70.0, 120.0);
        kp_angular = Eigen::Vector3d(650.0, 35.0, 1.0);
        kd_angular = Eigen::Vector3d(4.5, 4.5, 30.0);

        torques_gravity << 0.80, 0, 0, -0.80, 0, 0, 0.80, 0, 0, -0.80, 0, 0;
        joint_torques.setZero();

        power_level = 5;
    }

    // period of one gait cycle
    double plan_dt;
    int counter_per_plan;
    double counter_per_gait;
    double counter_per_swing;
    int counter;
    Eigen::Vector4d gait_counter;
    Eigen::Vector4d gait_counter_speed;
    
    // gait type
    int gait_type;
    int gait_type_last;

    // control target
    Eigen::Vector3d root_pos_d;
    Eigen::Vector3d root_euler_d;
    Eigen::Vector3d root_lin_vel_d;
    Eigen::Vector3d root_lin_vel_d_world;
    Eigen::Vector3d root_ang_vel_d;
    Eigen::Vector3d root_ang_vel_d_world;

    double robot_mass;

    Eigen::Matrix3d go1_trunk_inertia;
    Eigen::Matrix<double, 3, NUM_LEG> default_foot_pos;
    // important kinematics variables
    Eigen::Vector3d root_pos;
    Eigen::Quaterniond root_quat;
    Eigen::Vector3d root_euler;
    Eigen::Matrix3d root_rot_mat;
    Eigen::Matrix3d root_rot_mat_z;
    Eigen::Vector3d root_lin_vel;
    Eigen::Vector3d root_ang_vel;
    Eigen::Vector3d root_acc;

    Eigen::Vector4d foot_force;

    Eigen::Matrix<double, NUM_DOF, 1> joint_pos;
    Eigen::Matrix<double, NUM_DOF, 1> joint_vel;

    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_world; // in the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_abs;   // in a frame which centered at the robot frame's origin but parallels to the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_rel;   // in the robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_start;

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_world; // in the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs;   // in a frame which centered at the robot frame's origin but parallels to the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel;   // in the robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs_mpc;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel_last_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_last_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_recent_contact;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_world;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_abs;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_rel;
    Eigen::Matrix<double, 12, 12> j_foot;

    bool contacts[NUM_LEG];       // flag to decide leg in the stance/swing mode
    bool plan_contacts[NUM_LEG];  // planed flag for stance/swing mode
    bool early_contacts[NUM_LEG]; // true if foot hit objects during swing

    // controller variables
    double kp_lin_x;
    double kd_lin_x;
    double kf_lin_x;
    double kp_lin_y;
    double kd_lin_y;
    double kf_lin_y;

    Eigen::Matrix<double, NUM_DOF_PER_LEG, NUM_LEG> kp_foot;
    Eigen::Matrix<double, NUM_DOF_PER_LEG, NUM_LEG> kd_foot;
    Eigen::Matrix<double, NUM_DOF_PER_LEG, 1> km_foot;

    double kp_linear_lock_x, kp_linear_lock_y;
    Eigen::Vector3d kp_linear;
    Eigen::Vector3d kd_linear;
    Eigen::Vector3d kp_angular;
    Eigen::Vector3d kd_angular;

    Eigen::Matrix<double, NUM_DOF, 1> torques_gravity;
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;

    // IMU sensor data
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_ang_vel;

    // state estimation
    bool estimated_contacts[NUM_LEG]; // true if the estimator thinks the foot has contact
    Eigen::Vector3d estimated_root_pos;
    Eigen::Vector3d estimated_root_vel;

    // hardware
    int power_level;
};