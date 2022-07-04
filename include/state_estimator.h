#ifndef _STATE_ESTIMATOR_H_
#define _STATE_ESTIMATOR_H_

#include <iostream>

#include <ros/ros.h>

class StateEstimator{
private:


public:
    StateEstimator(ros::NodeHandle& nh);
    ~StateEstimator();
    

private:
    ros::NodeHandle nh_;
    
public:


};
#endif