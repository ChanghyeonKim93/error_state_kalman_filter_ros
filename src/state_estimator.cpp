#include "state_estimator.h"
StateEstimator::StateEstimator(ros::NodeHandle& nh)
: nh_(nh)
{
    ROS_INFO_STREAM("StateEstimator - starts");
};

StateEstimator::~StateEstimator(){
    ROS_INFO_STREAM("StateEstimator - terminated");
};