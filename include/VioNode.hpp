#ifndef VIO_NODE_HPP
#define VIO_NODE_HPP

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>

#include <Optimizer.hpp>
#include <Parameters.hpp>
#include <MultiDvo.hpp>
#include <Dvo.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include <eigen_conversions/eigen_msg.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>

#include <blackbird/MotorRPM.h>

class VioNode{
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VioNode(ros::NodeHandle& nh, const Parameters& p) : optimizer_(p){
            odom_pub = nh.advertise<nav_msgs::Odometry>("VIO_odom", 50);
            
            multi_dvo.reset(new MultiDvo(2, 2));
            multi_dvo->setNumMaxIter(10);
            multi_dvo->setNormThresh(1e-6);
            multi_dvo->setChangeThresh(1e-6);
            multi_dvo->setCostThresh(1e-5);

            std::string config;
            nh.getParam("dvo_config", config);
            dvo0.reset( new Dvo(config, 2, 2) );
            multi_dvo->addDvo(dvo0);
            T_cumulative_ = Eigen::Matrix4f::Identity();
            T_1prev_ = Eigen::Matrix4f::Identity();
            T_2prev_ = Eigen::Matrix4f::Identity();
            initialized_ = false;

            image_counter_ = 0;
        };

        ~VioNode(){};

        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

        void dynamicsCallback(const blackbird::MotorRPM::ConstPtr& msg);

        void imageCallback(const sensor_msgs::ImageConstPtr &cam0, const sensor_msgs::ImageConstPtr &cam1);

    private:

        ros::Publisher odom_pub;

        Optimizer optimizer_;

        uint64_t prev_imu_msg_time = 0;

        uint64_t prev_dynamics_msg_time = 0;

        std::shared_ptr<MultiDvo> multi_dvo;
        std::shared_ptr<Dvo> dvo0, dvo1;

        Eigen::Matrix4f T_cumulative_;
        Eigen::Matrix4f T_1prev_, T_2prev_; // const velocity model history

        bool initialized_;

        int image_counter_;

};

#endif
