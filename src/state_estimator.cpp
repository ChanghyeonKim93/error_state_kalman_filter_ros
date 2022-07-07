#include "state_estimator.h"
StateEstimator::StateEstimator(ros::NodeHandle& nh)
: nh_(nh)
{
    ROS_INFO_STREAM("StateEstimator - starts");

    // get parameters
    if(!ros::param::has("~topic_imu")) throw std::runtime_error("StateEstimator - no 'topic_imu' is set. terminate program.\n");
    ros::param::get("~topic_imu", topicname_imu_);

    if(!ros::param::has("~topic_mag")) throw std::runtime_error("StateEstimator - no 'topic_mag' is set. terminate program.\n");
    ros::param::get("~topic_mag", topicname_mag_);

    if(!ros::param::has("~topic_optitrack")) throw std::runtime_error("StateEstimator - no 'topic_optitrack' is set. terminate program.\n");
    ros::param::get("~topic_optitrack", topicname_optitrack_);
    
    if(!ros::param::has("~topic_nav_filtered")) throw std::runtime_error("StateEstimator - no 'topic_nav_filtered' is set. terminate programe.\n");
    ros::param::get("~topic_nav_filtered", topicname_nav_filtered_);
    
    if(!ros::param::has("~topic_nav_raw")) throw std::runtime_error("StateEstimator - no 'topic_nav_raw' is set. terminate programe.\n");
    ros::param::get("~topic_nav_raw", topicname_nav_raw_);
    
    ROS_INFO_STREAM("StateEstimator - ROS parameters are successfully got.\n");

    // Subscribing
    sub_imu_       = nh_.subscribe<sensor_msgs::Imu>
        (topicname_imu_,         1, &StateEstimator::callbackIMU, this);
    sub_mag_       = nh_.subscribe<sensor_msgs::MagneticField>
        (topicname_mag_,         1, &StateEstimator::callbackMag, this);
    sub_optitrack_ = nh_.subscribe<geometry_msgs::PoseStamped>
        (topicname_optitrack_,   1, &StateEstimator::callbackOptitrack, this);

    // Publishing 
    pub_nav_raw_      = nh_.advertise<nav_msgs::Odometry>
        (topicname_nav_raw_, 1);
    pub_nav_filtered_ = nh_.advertise<nav_msgs::Odometry>
        (topicname_nav_filtered_, 1);
        
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
    double t_now = imu_current_.header.stamp.toSec();

    Vec3 am;
    Vec3 wm;
    am << imu_current_.linear_acceleration.x, imu_current_.linear_acceleration.y, imu_current_.linear_acceleration.z;
    wm << imu_current_.angular_velocity.x, imu_current_.angular_velocity.y, imu_current_.angular_velocity.z;

    // Predict filter
    filter_->predict(am, wm, t_now);
    
    if( filter_->isInitialized() ){
        // publish the filtered data
        ESKF::NominalState X_nom;
        filter_->getFilteredStates(X_nom);
        
        // Fill the nav message
        nav_filtered_current_.header.frame_id = "world";
        ++nav_filtered_current_.header.seq;
        nav_filtered_current_.header.stamp = ros::Time::now();

        // Position and orientation
        X_nom.p = filter_->getFixedParameters().R_BI*X_nom.p;
        nav_filtered_current_.pose.pose.position.x = X_nom.p(0); // global
        nav_filtered_current_.pose.pose.position.y = X_nom.p(1);
        nav_filtered_current_.pose.pose.position.z = X_nom.p(2);

        X_nom.q = geometry::q1_mult_q2(
                    geometry::q1_mult_q2(filter_->getFixedParameters().q_IB, X_nom.q),
                    filter_->getFixedParameters().q_BI);

        nav_filtered_current_.pose.pose.orientation.w = X_nom.q(0); // global
        nav_filtered_current_.pose.pose.orientation.x = X_nom.q(1);
        nav_filtered_current_.pose.pose.orientation.y = X_nom.q(2);
        nav_filtered_current_.pose.pose.orientation.z = X_nom.q(3);

        // Velocities
        X_nom.v = filter_->getFixedParameters().R_BI*X_nom.v;
        nav_filtered_current_.twist.twist.linear.x = X_nom.v(0); // body
        nav_filtered_current_.twist.twist.linear.y = X_nom.v(1);
        nav_filtered_current_.twist.twist.linear.z = X_nom.v(2);

        Vec3 wm;
        wm << imu_current_.angular_velocity.x, imu_current_.angular_velocity.y, imu_current_.angular_velocity.z;
        wm = wm - X_nom.bg;
        wm = filter_->getFixedParameters().R_BI*wm; // body
        nav_filtered_current_.twist.twist.angular.x = imu_current_.angular_velocity.x;
        nav_filtered_current_.twist.twist.angular.y = imu_current_.angular_velocity.y;
        nav_filtered_current_.twist.twist.angular.z = imu_current_.angular_velocity.z;

        pub_nav_filtered_.publish(nav_filtered_current_);
    }
};


void StateEstimator::callbackMag(const sensor_msgs::MagneticFieldConstPtr& msg){
    mag_current_ = *msg;
    // std::cout << mag_current_.header.seq << ", Mag Gets: " << mag_current_.header.stamp.toNSec() << std::endl;
    
    // filter_->updateMagnetometer();
};

void StateEstimator::callbackOptitrack(const geometry_msgs::PoseStampedConstPtr& msg){
    optitrack_current_ = *msg;
    double t_now = optitrack_current_.header.stamp.toSec();

    Vec3 p_observe;
    Vec4 q_observe;
    p_observe << optitrack_current_.pose.position.x, optitrack_current_.pose.position.y, optitrack_current_.pose.position.z;
    q_observe << optitrack_current_.pose.orientation.w, optitrack_current_.pose.orientation.x, optitrack_current_.pose.orientation.y, optitrack_current_.pose.orientation.z;

    // Update filter
    filter_->updateOptitrack(p_observe, q_observe, t_now);
    
    // Fill the nav message
    nav_raw_current_.header.frame_id = "world";
    ++nav_raw_current_.header.seq;
    nav_raw_current_.header.stamp = ros::Time::now();
    
    // Position and orientation
    nav_raw_current_.pose.pose.position.x = optitrack_current_.pose.position.x;
    nav_raw_current_.pose.pose.position.y = optitrack_current_.pose.position.y;
    nav_raw_current_.pose.pose.position.z = optitrack_current_.pose.position.z;

    nav_raw_current_.pose.pose.orientation.w = optitrack_current_.pose.orientation.w;
    nav_raw_current_.pose.pose.orientation.x = optitrack_current_.pose.orientation.x;
    nav_raw_current_.pose.pose.orientation.y = optitrack_current_.pose.orientation.y;
    nav_raw_current_.pose.pose.orientation.z = optitrack_current_.pose.orientation.z;

    // Velocities
    nav_raw_current_.twist.twist.linear.x = 0;
    nav_raw_current_.twist.twist.linear.y = 0;
    nav_raw_current_.twist.twist.linear.z = 0;

    nav_raw_current_.twist.twist.angular.x = imu_current_.angular_velocity.x;
    nav_raw_current_.twist.twist.angular.y = imu_current_.angular_velocity.y;
    nav_raw_current_.twist.twist.angular.z = imu_current_.angular_velocity.z;

    pub_nav_raw_.publish(nav_raw_current_);
};