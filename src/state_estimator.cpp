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
    
    double t_now = imu_current_.header.stamp.toSec();

    Vec3 am(imu_current_.linear_acceleration.x, imu_current_.linear_acceleration.y, imu_current_.linear_acceleration.z);
    Vec3 wm(imu_current_.angular_velocity.x, imu_current_.angular_velocity.y, imu_current_.angular_velocity.z);

    filter_->predict(am, wm, t_now);
};


void StateEstimator::callbackMag(const sensor_msgs::MagneticFieldConstPtr& msg){
    mag_current_ = *msg;
    std::cout << mag_current_.header.seq << ", Mag Gets: " << mag_current_.header.stamp.toNSec() << std::endl;
    
    // filter_->updateMagnetometer();
};

void StateEstimator::callbackOptitrack(const geometry_msgs::PoseStampedConstPtr& msg){
    optitrack_current_ = *msg;
    std::cout << optitrack_current_.header.seq << ", Optitrack Gets: " << optitrack_current_.header.stamp.toNSec() << std::endl;

    Vec3 p_observe(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Vec4 q_observe(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    filter_->updateOptitrack(p_observe, q_observe);
};