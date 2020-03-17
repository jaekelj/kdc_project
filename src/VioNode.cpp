#include <VioNode.hpp>


void VioNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    // std::cout << "In IMU Callback" << std::endl;
    Eigen::Vector3d linear_acc, angular_vel;
    Eigen::Quaterniond ahrs; 
    tf::vectorMsgToEigen (msg->linear_acceleration, linear_acc);
    tf::vectorMsgToEigen (msg->angular_velocity, angular_vel);
    tf::quaternionMsgToEigen (msg->orientation, ahrs);

    uint64_t msg_time = msg->header.stamp.toNSec();

    if (prev_imu_msg_time == 0){
        prev_imu_msg_time = msg_time;
        return;
    }

    Eigen::Matrix<double,7,1> imuMeasurement;
    imuMeasurement << (msg_time - prev_imu_msg_time)/1e9, linear_acc , angular_vel; //TODO replace with IMU period

    std::pair<uint64_t,Eigen::Matrix<double,7,1>> msg_to_push(msg_time, imuMeasurement);
    prev_imu_msg_time = msg_time;
    optimizer_.addImuMeasurement(msg_to_push);
}

void VioNode::imageCallback(const sensor_msgs::Image::ConstPtr& msg){ // If we want to use stereo images we should change this functions to take in 2 images
    // Initialize the optimizer on the first image
    if (!optimizer_.isInitialized()){
        optimizer_.initializeGraph(msg->header.stamp.toNSec());
        optimizer_.setInitialTime(msg->header.stamp.toNSec());
        optimizer_.startThread();
        return;
    }

    // TODO: Process image, pass features to optimizer
}


int main(int argc, char **argv){
    ros::init(argc, argv, "vio");   
    ros::NodeHandle nh("~"); 

    VioNode vio(nh);

    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu0", 1000);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh,"/cam0/image_raw",10);
    imu_sub.registerCallback(&VioNode::imuCallback, &vio);
    image_sub.registerCallback(&VioNode::imageCallback, &vio);
    ros::spin();
}