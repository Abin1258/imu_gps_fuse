#ifndef IMU_GPS_FUSE_IMU_GPS_FUSE_HPP
#define IMU_GPS_FUSE_IMU_GPS_FUSE_HPP

#include "ros/ros.h"
#include "ros/time.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>
#include <cmath>

#include "imu_gps_localizer/base_type.h"
#include "imu_gps_localizer/imu_gps_localizer.h"

using namespace ImuGpsLocalization;
using namespace std;

class ImuGpsFuse
{
private:
    ros::Subscriber imusub;
    ros::Subscriber navsatsub;
    ros::Publisher odompub;
    ros::Publisher pathpub;

    nav_msgs::Path gipath;
    nav_msgs::Odometry giodom;

    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_fuse_ptr;

public:
    ImuGpsFuse(ros::NodeHandle& nh);
    ~ImuGpsFuse();

    void NavsatCallback(const sensor_msgs::NavSatFixConstPtr& navsat_msg);
    void ImuCallback(const sensor_msgs::ImuConstPtr& IMU_msg);
    void ConvertStateToRosTopic(const ImuGpsLocalization::State& state);

};

ImuGpsFuse::ImuGpsFuse(ros::NodeHandle& nh){
    //收发初始化
    imusub = nh.subscribe("/imu/data", 32,  &ImuGpsFuse::ImuCallback, this);
    navsatsub = nh.subscribe("/gps/fix", 32, &ImuGpsFuse::NavsatCallback, this);
    odompub = nh.advertise<nav_msgs::Odometry>("gi_odom", 32);
    pathpub = nh.advertise<nav_msgs::Path>("/gi_path", 32);
    //IMU偏差和重力项初始化
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
    nh.param("acc_noise",       acc_noise, 1e-2);
    nh.param("gyro_noise",      gyro_noise, 1e-4);
    nh.param("acc_bias_noise",  acc_bias_noise, 1e-6);
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);
    double x, y, z;
    nh.param("I_p_Gps_x", x, 0.);
    nh.param("I_p_Gps_y", y, 0.);
    nh.param("I_p_Gps_z", z, 0.);
    const Eigen::Vector3d I_p_Gps(x, y, z);

    //imu gps fuse 指针初始化
    imu_gps_fuse_ptr = std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(acc_noise, 
                                        gyro_noise, acc_bias_noise, gyro_bias_noise, I_p_Gps);
}


void ImuGpsFuse::NavsatCallback(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {
    // gps 数据好坏判断.
    if (navsat_msg->status.service <= 21) {
        std::cout << "[GpsCallBack]: Bad gps message!";
        return;
    }

    ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsPositionData>();
    gps_data_ptr->timestamp = navsat_msg->header.stamp.toSec();
    gps_data_ptr->lla << navsat_msg->latitude,
                         navsat_msg->longitude,
                         navsat_msg->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(navsat_msg->position_covariance.data());

    imu_gps_fuse_ptr->ProcessGpsPositionData(gps_data_ptr);
}

void ImuGpsFuse::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg){
    ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg->linear_acceleration.x, 
                         imu_msg->linear_acceleration.y,
                         imu_msg->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg->angular_velocity.x,
                          imu_msg->angular_velocity.y,
                          imu_msg->angular_velocity.z;
    
    ImuGpsLocalization::State fused_state;
    const bool ok = imu_gps_fuse_ptr->ProcessImuData(imu_data_ptr, &fused_state);
    if (!ok) {
        return;
    }

    // Publish fused state.
    ConvertStateToRosTopic(fused_state);
    pathpub.publish(gipath);
}

void ImuGpsFuse::ConvertStateToRosTopic(const ImuGpsLocalization::State& state) {
    gipath.header.frame_id = "path";
    gipath.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = gipath.header;

    pose.pose.position.x = state.G_p_I[0];
    pose.pose.position.y = state.G_p_I[1];
    pose.pose.position.z = state.G_p_I[2];

    const Eigen::Quaterniond G_q_I(state.G_R_I);
    pose.pose.orientation.x = G_q_I.x();
    pose.pose.orientation.y = G_q_I.y();
    pose.pose.orientation.z = G_q_I.z();
    pose.pose.orientation.w = G_q_I.w();

    gipath.poses.push_back(pose);
}

#endif