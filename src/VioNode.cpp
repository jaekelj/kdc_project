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
    Eigen::Vector4d rotor_rpm;

    for (int i = 0; i < msg->rpm.size(); ++i) {
      rotor_rpm[i] = msg->rpm[i];
    }

    uint64_t msg_time = msg->header.stamp.toNSec();

    if (prev_dynamics_msg_time == 0){
        prev_dynamics_msg_time = msg_time;
        return;
    }

    Eigen::Matrix<double,5,1> dynamicsMeasurement;
    dynamicsMeasurement << (msg_time - prev_dynamics_msg_time) / 1e9, rotor_rpm[0],
                            rotor_rpm[1], rotor_rpm[2], rotor_rpm[3];

    std::pair<uint64_t,Eigen::Matrix<double,5,1>> msg_to_push(msg_time, dynamicsMeasurement);
    prev_dynamics_msg_time = msg_time;
    optimizer_.addDynamicsMeasurement(msg_to_push);
}

void VioNode::rotorsCallback(const mav_msgs::Actuators::ConstPtr& msg){
    // std::cout << "In Rotors Callback" << std::endl;
    Eigen::Vector4d rotor_rpm;

    for (int i = 0; i < msg->angular_velocities.size(); ++i) {
      rotor_rpm[i] = msg->angular_velocities[i] * 9.5493; // convert to rpm TODO check
    }

    uint64_t msg_time = msg->header.stamp.toNSec();

    if (prev_rotors_msg_time == 0){
        prev_rotors_msg_time = msg_time;
        return;
    }

    Eigen::Matrix<double,5,1> rotorsMeasurement;
    rotorsMeasurement << (msg_time - prev_rotors_msg_time) / 1e9, rotor_rpm[0],
                            rotor_rpm[1], rotor_rpm[2], rotor_rpm[3];

    std::pair<uint64_t,Eigen::Matrix<double,5,1>> msg_to_push(msg_time, rotorsMeasurement);
    prev_rotors_msg_time = msg_time;
    optimizer_.addRotorsMeasurement(msg_to_push);
}

void VioNode::imageCallback(const sensor_msgs::ImageConstPtr &cam0, const sensor_msgs::ImageConstPtr &cam1)
{
    image_counter_++;
    if (image_counter_ % 3 == 0){
        return;
    }
    while (optimizer_.odom_buffer_.size() != 0){
        odom_pub.publish(optimizer_.odom_buffer_[0]);
        optimizer_.odom_buffer_.pop_front();
    }

    cv_bridge::CvImagePtr cam0_ptr = cv_bridge::toCvCopy(cam0);
    cv_bridge::CvImagePtr cam1_ptr = cv_bridge::toCvCopy(cam1);

    if (!optimizer_.isInitialized()){
        if(optimizer_.getImuBufferSize() != 0){
            optimizer_.initializeGraph(cam0->header.stamp.toNSec());
            optimizer_.setInitialTime(cam0->header.stamp.toNSec());
            optimizer_.startThread();
        }
        return;
    }

    if (initialized_) {
        // multi_dvo->setInitTransform( T_1prev_*T_2prev_*(T_1prev_.inverse()) );
        multi_dvo->setInitTransform(Eigen::Matrix4f::Identity());
        std::vector<cv::Mat> inputs;
        inputs.push_back(cam0_ptr->image);
        inputs.push_back(cam1_ptr->image);
        multi_dvo->track(inputs);
    }

    Eigen::Matrix4f T_curr_prev = multi_dvo->getRelativePose();
    T_cumulative_ = T_cumulative_*T_curr_prev.inverse();

    Eigen::Quaternion<float> q;
    Eigen::Matrix3f rotm = T_curr_prev.topLeftCorner(3,3);
    q = Eigen::Quaternion<float>(rotm);

    nav_msgs::Odometry odom_msg;
    odom_msg.pose.pose.position.x = T_curr_prev(0,3);
    odom_msg.pose.pose.position.y = T_curr_prev(1,3);
    odom_msg.pose.pose.position.z = T_curr_prev(2,3);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.header.stamp = cam0 -> header.stamp;

    Eigen::Matrix6f cov_mat = multi_dvo -> getCovariance();
    for (int i = 0; i < 6; i++){
        for (int j = 0; j < 6; j++){
            odom_msg.pose.covariance[i*6 + j] = cov_mat(i,j);
        }
    }

    dvo0->setReference(cam0_ptr->image, cam1_ptr->image);

    // update history for motion model
    T_2prev_ = T_1prev_;
    T_1prev_ = T_curr_prev;

    initialized_ = true;

    std::pair<uint64_t, geometry_msgs::PoseWithCovariance> msg_to_push(odom_msg.header.stamp.toNSec(), odom_msg.pose);
    optimizer_.addImageMeasurement(msg_to_push);
}


int main(int argc, char **argv){
    std::cout << "start" << std::endl;

    ros::init(argc, argv, "vio");
    ros::NodeHandle nh("~");

    ROS_INFO_STREAM("initializing");
    std::cout << "test" << std::endl;

    std::string file_path;
    nh.getParam("config_file_path", file_path);
    Parameters p;
    p.readConfig(file_path);

    VioNode vio(nh,p);

    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, p.imu_topic, 1000);
    imu_sub.registerCallback(&VioNode::imuCallback, &vio);

    message_filters::Subscriber<blackbird::MotorRPM> dynamics_sub(nh, p.motors_topic, 1000);
    dynamics_sub.registerCallback(&VioNode::dynamicsCallback, &vio);

    message_filters::Subscriber<mav_msgs::Actuators> rotors_sub(nh, p.rotors_topic, 1000);
    rotors_sub.registerCallback(&VioNode::rotorsCallback, &vio);

    message_filters::Subscriber<sensor_msgs::Image> image_subL(nh, p.left_image_topics[0],10);
    message_filters::Subscriber<sensor_msgs::Image> image_subR(nh, p.right_image_topics[0],10);
    std::cout << "subscribing to " << p.left_image_topics[0] << " and " << p.right_image_topics[0] << " and " <<  p.rotors_topic << " and " << p.imu_topic << std::endl;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol_multi;
    message_filters::Synchronizer<sync_pol_multi> sync_multi(sync_pol_multi(1000), image_subL, image_subR);
    sync_multi.registerCallback(boost::bind(&VioNode::imageCallback, &vio, _1, _2));

    ros::spin();
}
