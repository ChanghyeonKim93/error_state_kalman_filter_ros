#ifndef _STATE_ESTIMATOR_H_
#define _STATE_ESTIMATOR_H_

#include <iostream>
#include <string>

#include <Eigen/Dense>

#include <ros/ros.h>

// Subscribing messages
#include <sensor_msgs/Imu.h> // Imu (acc & gyro)
#include <sensor_msgs/MagneticField.h> // Imu magnetometer
#include <geometry_msgs/PoseStamped.h> // optitrack

// Publishing messages
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// Error state kalman filter.
#include <error_state_kalman_filter.h>

#include "timer.h"

using namespace Eigen;

#define FilterType ESKF

class StateEstimator{
// Callback functions
private:
	void callbackOptitrack(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackIMU(const sensor_msgs::ImuConstPtr& msg);
	void callbackMag(const sensor_msgs::MagneticFieldConstPtr& msg);

    void run();

public:
    StateEstimator(ros::NodeHandle& nh);
    ~StateEstimator();
    
private:
    ros::NodeHandle nh_;

	// subscribers
    std::string topicname_imu_;
    std::string topicname_mag_;
    std::string topicname_optitrack_;

	ros::Subscriber sub_imu_;
	ros::Subscriber sub_mag_;
	ros::Subscriber sub_optitrack_;

    sensor_msgs::Imu           imu_current_;
    sensor_msgs::MagneticField mag_current_;
    geometry_msgs::PoseStamped optitrack_current_;

    // publishers
    std::string topicname_nav_raw_;
    std::string topicname_nav_filtered_;

    ros::Publisher pub_nav_raw_;
    ros::Publisher pub_nav_filtered_;
    
    nav_msgs::Odometry nav_raw_current_;
    nav_msgs::Odometry nav_filtered_current_;

// Filter
private:   
    std::unique_ptr<FilterType> filter_;
};
#endif