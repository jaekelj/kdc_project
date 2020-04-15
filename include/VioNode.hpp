#ifndef VIO_NODE_HPP
#define VIO_NODE_HPP

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>

#include "Optimizer.hpp"
#include <Parameters.hpp>
#include <FeatureHandler.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include <eigen_conversions/eigen_msg.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <opencv2/core/eigen.hpp>



class VioNode{
    public:
        VioNode(ros::NodeHandle& nh, const Parameters& p) : feature_handler_(p), optimizer_(p){
            odom_pub = nh.advertise<nav_msgs::Odometry>("VIO_odom", 50);
        };

        ~VioNode(){};

        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

        void imageCallback(const sensor_msgs::ImageConstPtr &cam0, const sensor_msgs::ImageConstPtr &cam1);
    
    private:

        ros::Publisher odom_pub;

        Optimizer optimizer_;

        FeatureHandler feature_handler_;

        uint64_t prev_imu_msg_time = 0;

};

#endif