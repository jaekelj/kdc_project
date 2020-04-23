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
    imuMeasurement << (msg_time - prev_imu_msg_time) / 1e9, linear_acc , angular_vel; //TODO replace with IMU period

    std::pair<uint64_t,Eigen::Matrix<double,7,1>> msg_to_push(msg_time, imuMeasurement);
    prev_imu_msg_time = msg_time;
    optimizer_.addImuMeasurement(msg_to_push);
}

void VioNode::dynamicsCallback(const blackbird::MotorRPM::ConstPtr& msg){
    // std::cout << "In Dynamics  Callback" << std::endl;
    // std::vector<float> rotor_rpm = msg->rpm;

    // Eigen::Vector4d rotor_rpm(msg->rpm.data());
    Eigen::VectorXf rotor_rpm = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(msg->rpm.data(), msg->rpm.size());

    uint64_t msg_time = msg->header.stamp.toNSec();

    if (prev_dynamics_msg_time == 0){
        prev_dynamics_msg_time = msg_time;
        return;
    }

    Eigen::Matrix<double,5,1> dynamicsMeasurement;
    dynamicsMeasurement << (msg_time - prev_dynamics_msg_time) / 1e9, rotor_rpm;

    std::pair<uint64_t,Eigen::Matrix<double,5,1>> msg_to_push(msg_time, dynamicsMeasurement);
    prev_dynamics_msg_time = msg_time;
    optimizer_.addDynamicsMeasurement(msg_to_push);
}

void VioNode::imageCallback(const sensor_msgs::ImageConstPtr &cam0, const sensor_msgs::ImageConstPtr &cam1)
{
    // std::cout << " "
    cv_bridge::CvImageConstPtr cam0_ptr = cv_bridge::toCvCopy(cam0, sensor_msgs::image_encodings::MONO8);
    cv_bridge::CvImageConstPtr cam1_ptr = cv_bridge::toCvCopy(cam1, sensor_msgs::image_encodings::MONO8);

   // Initialize the optimizer on the first image
    if (!optimizer_.isInitialized()){
        optimizer_.initializeGraph(cam0->header.stamp.toNSec());
        optimizer_.setInitialTime(cam0->header.stamp.toNSec());
        optimizer_.startThread();
        feature_handler_.initializeFeatures(cam0_ptr->image, cam1_ptr->image, 0);
    }
    else
    {
        feature_handler_.temporalMatch(cam0_ptr->image, cam1_ptr->image, cam0->header.stamp.toNSec(), 0);
        feature_handler_.removeRedundantFeatures();
        feature_handler_.addFeatures(cam0_ptr->image, cam1_ptr->image, 0);
        optimizer_.addImageMeasurement(std::make_pair(cam0->header.stamp.toNSec(),feature_handler_.sendToRANSAC(cam0, cam1)));
        // feature_handler_.sendToRANSAC(cam0, cam1);
    }
    feature_handler_.setPrevImage(cam0_ptr->image);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "vio");
    ros::NodeHandle nh("~");

    std::string file_path;
    nh.getParam("config_file_path", file_path);
    Parameters p;
    p.readConfig(file_path);

    VioNode vio(nh,p);

    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, p.imu_topic, 1000);
    imu_sub.registerCallback(&VioNode::imuCallback, &vio);

    message_filters::Subscriber<sensor_msgs::Image> image_subL(nh, p.left_image_topics[0],10);
    message_filters::Subscriber<sensor_msgs::Image> image_subR(nh, p.right_image_topics[0],10);
    std::cout << "subscribing to " << p.left_image_topics[0] << " and " << p.right_image_topics[0] << std::endl;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol_multi;
    message_filters::Synchronizer<sync_pol_multi> sync_multi(sync_pol_multi(1000), image_subL, image_subR);
    sync_multi.registerCallback(boost::bind(&VioNode::imageCallback, &vio, _1, _2));

    ros::spin();
}
