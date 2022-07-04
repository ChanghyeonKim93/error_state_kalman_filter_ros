#include <iostream>
#include <ros/ros.h>

#include "state_estimator.h"

int main(int argc, char **argv) {
    // ros::init(argc, argv, "hce_gcs", ros::init_options::NoSigintHandler);
    // SignalHandle::initSignalHandler();
    ros::init(argc, argv, "eskf_based_approach");

    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("eskf_based_approach - STARTS.");

	try{
		if(ros::ok()){
			std::unique_ptr<StateEstimator> st_est;
			st_est = std::make_unique<StateEstimator>(nh);			
		}
		else{
			throw std::runtime_error("ROS not ok");
		}
	}
	catch (std::exception& e){
        ROS_ERROR(e.what());
	}

    ROS_INFO_STREAM("eskf_based_approach - TERMINATED.");
	return 0;
}