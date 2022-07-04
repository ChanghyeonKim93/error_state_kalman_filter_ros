#include "state_estimator.h"
StateEstimator::StateEstimator(ros::NodeHandle& nh)
: nh_(nh)
{
    ROS_INFO_STREAM("StateEstimator - starts");

    // get parameters
    if(!ros::param::has("~topic_imu")) throw std::runtime_error("TopicLoggerNew - no 'topic_imu' is set. terminate program.\n");
    ros::param::get("~topic_imu", topicname_imu_);

    if(!ros::param::has("~topic_mag")) throw std::runtime_error("TopicLoggerNew - no 'topic_mag' is set. terminate program.\n");
    ros::param::get("~topic_mag", topicname_mag_);

    if(!ros::param::has("~topic_optitrack")) throw std::runtime_error("TopicLoggerNew - no 'topic_optitrack' is set. terminate program.\n");
    ros::param::get("~topic_optitrack", topicname_optitrack_);

    ROS_INFO_STREAM("StateEstimator - ROS parameters are successfully got.\n");

    // Subscribing
    sub_imu_       = nh_.subscribe<sensor_msgs::Imu>
        (topicname_imu_,         10, &StateEstimator::callbackIMU, this);
    sub_mag_       = nh_.subscribe<sensor_msgs::MagneticField>
        (topicname_mag_,         10, &StateEstimator::callbackMag, this);
    sub_optitrack_ = nh_.subscribe<geometry_msgs::PoseStamped>
        (topicname_optitrack_,   10, &StateEstimator::callbackOptitrack, this);

    // Filter generation
    filter_ = std::make_unique<FilterType>();

    // run
    this->run();
};

StateEstimator::~StateEstimator(){
    ROS_INFO_STREAM("StateEstimator - terminated");
};

void StateEstimator::run(){
    ROS_INFO_STREAM("StateEstimator - runs at [" << 1000 <<"] Hz.");
    ros::Rate rate(1000);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
};


void StateEstimator::callbackIMU(const sensor_msgs::ImuConstPtr& msg){
    imu_current_ = *msg;
    std::cout << imu_current_.header.seq << ", IMU Gets: " << imu_current_.header.stamp.toNSec() << std::endl;


    filter_->propagate();

    // msg->header.seq << " ";
    // (double)msg->header.stamp.toNSec()*0.000000001 << " ";
    // (double)msg->linear_acceleration.x << " ";
    // (double)msg->linear_acceleration.y << " ";
    // (double)msg->linear_acceleration.z << " ";
    // (double)msg->angular_velocity.x << " ";
    // (double)msg->angular_velocity.y << " ";
    // (double)msg->angular_velocity.z << " ";
    // (double)msg->orientation.w << " ";
    // (double)msg->orientation.x << " ";
    // (double)msg->orientation.y << " ";
    // (double)msg->orientation.z << "\n";
};


void StateEstimator::callbackMag(const sensor_msgs::MagneticFieldConstPtr& msg){
    mag_current_ = *msg;
    std::cout << mag_current_.header.seq << ", Mag Gets: " << mag_current_.header.stamp.toNSec() << std::endl;
    
    // msg->header.seq << " ";
    // (double)msg->header.stamp.toNSec()*0.000000001 << " ";
    // (double)msg->magnetic_field.x << " ";
    // (double)msg->magnetic_field.y << " ";
    // (double)msg->magnetic_field.z << "\n";
};

void StateEstimator::callbackOptitrack(const geometry_msgs::PoseStampedConstPtr& msg){
    optitrack_current_ = *msg;
    std::cout << optitrack_current_.header.seq << ", Optitrack Gets: " << optitrack_current_.header.stamp.toNSec() << std::endl;

    filter_->update();
    
    // msg->header.seq << " ";
    // (double)msg->header.stamp.toNSec()*0.000000001 << " ";
    // (double)msg->pose.position.x << " ";
    // (double)msg->pose.position.y << " ";
    // (double)msg->pose.position.z << " ";
    // (double)msg->pose.orientation.w << " ";
    // (double)msg->pose.orientation.x << " ";
    // (double)msg->pose.orientation.y << " ";
    // (double)msg->pose.orientation.z << "\n";
};