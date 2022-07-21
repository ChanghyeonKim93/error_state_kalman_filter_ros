# State Estimator (Fusing Mocap System and IMU)

This package is an '**Error State Kalman Filter (ESKF)**-based state estimator'. 

It integrates:

* 10 ~ 100 Hz motion capture system data (Vicon or Optitrack, 3D position & 3D orientation of an object) 

* 100 ~ 1000 Hz IMU data (3D body acc. & 3D body angular vel.).

It gives estimated states at IMU data frequency. The states include 15 states:

* 3D global positions of the body (3D vector)
* 3D global velocities of the body (3D vector)
* 3D global orientation of the body (quaternion)
* 3D IMU accelerometer bias estimation w.r.t. the IMU frame
* 3D IMU gyroscope bias estimation w.r.t. the IMU frame

Tested in:

* Ubuntu 20.04LTS with ROS Noetic

1.Dependencies
------
* rqt-multiplot : it visualizes the estimated states and the observations from the Vicon(or optitrack) in **real-time.**

Install the rqt-multiplot by the below command:

    sudo apt-get install ros-{YOUR_DISTRO}-rqt-multiplot


2.Installation
------
    cd ~/{$YOUR_WORKSPACE}/src

    git clone https://github.com/ChanghyeonKim93/state_estimator.git

    cd .. && catkin_make (or catkin build state_estimator)
    

3.Usage
------
    roslaunch state_estimator optitrack_vn100t.launch 
    
    
4.Parameters
------
Before using this package, you should make sure user parameters to fit to your motion capture system spec. and IMU spec. in ros launch file.
Among the parameters, **IMU noise** parameters very critically effects on the overall estimation performance. Please find the best parameters for your system by simulations.

In addition, you should determine the **BODY TO IMU ROTATION (R_BI)** because the estimated states are represented in the **BODY FRAME**. If you just want to express the estimation values in the IMU frame, set the **BODY TO IMU ROTATION (R_BI)** with 3 x 3 identity matrix.

If you don't know the IMU acc. & gyro. bias values, just set (0,0,0). Note that the better accuracy of the initial bias values, the more stable sensor fusion results.

- `topic_imu`: your IMU topic name to subscribe. (message type: **sensor_msgs::Imu**) 
- `topic_mag`: your Magnetometer topic name to subscribe. (message type: **sensor_msgs::MagneticField**. Until now, this is not used in this package.) 
- `topic_optitrack`: your Motion Capture System topic name to subscribe. (message type: **geometry_msgs::PoseStamped**) 

- `topic_nav_raw`: motion capture data name converted to the nav_msgs::Odometry to rviz visualization. (message type: **nav_msgs::Odometry**) 
- `topic_nav_filtered`: estimation result topic name to publish. (message type: **nav_msgs::Odometry**) 

- `noise_accel`: IMU accelerometer noise
- `noise_gyro`: IMU gyroscope noise
- `noise_mag`: IMU magnetometer noise (not used now)

- `acc_bias`: IMU accelerometer bias (If you don't know, set 0,0,0)
- `gyro_bias`: IMU gyroscope bias( If you don't know, set 0,0,0)
- `mag_bias`: IMU magnetometer bias (not used now)

- `noise_optitrack_position`: motion capture system position noise (meters). If observations of the mocap system is very noisy and fluctuating, set larger than 0.01 meters.
- `noise_optitrack_orientation`: motion capture system orientation noise (radians). If observations of the mocap system is very noisy and fluctuating, set larger than 0.01 radians.

- `R_BI`: 3 x 3 rotation matrix on SO(3). A fixed rotation from the BODY frame (ex. drone) to the IMU frame. The identity is [1,0,0,0,1,0,0,0,1].

