#include "Dvo.hpp"

Dvo::Dvo(std::string config_file, int num_scales, int first_scale) {
           
  readParameters(config_file, num_scales, first_scale);

  J_.resize(num_scales);
  refVal_.resize(num_scales);
  refPoints_.resize(num_scales);
  H_.resize(num_scales);

  num_scales_ = num_scales;
  first_scale_ = first_scale;

  // defaults
  selection_region_size_ = 3;
  pixel_selection_scale_ = 100;

}

void Dvo::setInput(const cv::Mat &left) {
  cv::Mat left_rect;
  remap(left, left_rect, map0x_, map0y_, CV_INTER_LINEAR);
  input_pyr_ =
    imageProcessing::createImagePyramid(left_rect, num_scales_, first_scale_);
}

void Dvo::setReference(const cv::Mat &left, const cv::Mat &right) {
  cv::Mat left_rect, right_rect;
  remap(left, left_rect, map0x_, map0y_, CV_INTER_LINEAR);
  remap(right, right_rect, map1x_, map1y_, CV_INTER_LINEAR);
  cv::Mat disparity = bm_->compute(left_rect, right_rect);
  cv::imshow("test", visualizeDepthOnReference(left_rect, disparity, num_scales_ - 1));
  cv::waitKey(1);
  std::vector<cv::Mat> ref_pyr, ref_disparity;
  imageProcessing::createImageAndDisparityPyramid(left_rect,
                                                  disparity,
                                                  num_scales_,
                                                  first_scale_,
                                                  ref_pyr,
                                                  ref_disparity);

  // get reference data
  for (int scale = 0; scale < num_scales_; scale++) {
    constructReference(ref_pyr[scale], ref_disparity[scale], scale);
  }
}

void Dvo::constructReference(const cv::Mat &gray,
                             const cv::Mat &disparity,
                             int scale) {

  cv::Mat grad_x, grad_y, grad_mag;
  imageProcessing::getGradients(gray, grad_x, grad_y, grad_mag);

  std::vector<cv::Point> imagePoints;
  if (scale >= pixel_selection_scale_) {
    imagePoints = pixelSelection::nonmaxSuppression(grad_mag,
                                                    disparity,
                                                    selection_region_size_);
  } else {
    cv::findNonZero((disparity > 0.0), imagePoints);
  }
  // std::cout << "image size: " << gray.rows << " " << gray.cols << std::endl;
  // std::cout << "number of Points: " << imagePoints.size() << std::endl;

  J_[scale].resize(imagePoints.size(), 6);
  refVal_[scale].resize(imagePoints.size());
  refPoints_[scale].resize(4, imagePoints.size());
  H_[scale].clear();
  H_[scale].reserve(imagePoints.size());

  float fx = K_[scale](0, 0);
  float fy = K_[scale](1, 1);
  float fx_inv = 1 / fx;
  float fy_inv = 1 / fy;
  float cx = K_[scale](0, 2);
  float cy = K_[scale](1, 2);
  float bfx = baseline_[scale] * fx;
  for (int i = 0; i < imagePoints.size(); i++) {
    int v = imagePoints[i].y;
    int u = imagePoints[i].x;
    float z = bfx / disparity.at<float>(v, u);
    float z_inv = 1 / z;
    float x = z * (u - cx) * fx_inv;
    float y = z * (v - cy) * fy_inv;
    Eigen::Vector4f p_body = T_b_c_ * Eigen::Vector4f(x, y, z, 1);

    Eigen::RowVector2f grad(grad_x.at<float>(v, u), grad_y.at<float>(v, u));
    Eigen::Matrix<float, 2, 3> proj;
    proj << fx, 0, -fx * x * z_inv,
      0, fy, -fy * y * z_inv;
    Eigen::Matrix<float, 3, 6> tf;
    tf << 0, p_body(2), -p_body(1), 1, 0, 0,
      -p_body(2), 0, p_body(0), 0, 1, 0,
      p_body(1), -p_body(0), 0, 0, 0, 1;

    J_[scale].row(i) = z_inv * grad * proj * R_c_b_ * tf;
    H_[scale].push_back(J_[scale].row(i).transpose() * J_[scale].row(i));
    refVal_[scale](i) = gray.at<uchar>(v, u);
    refPoints_[scale].col(i) = p_body;
  }
  //cv::imshow("g",gray);
  //cv::waitKey();
}

void Dvo::constructSystem(const Eigen::Matrix4f &T, int scale) {

  Eigen::Matrix4Xf inputPoints = T_c_b_ * T * refPoints_[scale];
  Eigen::Matrix3Xf projections = K_[scale] * inputPoints.topRows(3);
  Eigen::Matrix2Xf p = projections.colwise().hnormalized();

  A_ = Eigen::Matrix6f::Zero();
  b_ = Eigen::Vector6f::Zero();
  cost_sum_ = 0;
  for (int i = 0; i < p.cols(); i++) {
    if ((p(0, i) > 0) && (p(0, i) < input_pyr_[scale].cols - 1) &&
      (p(1, i) > 0) && (p(1, i) < input_pyr_[scale].rows - 1)) {
      float error =
        utils::bilinearInterpolation(input_pyr_[scale], p(0, i), p(1, i))
          - refVal_[scale](i);
      // A_.noalias() += J_[scale].row(i).transpose()*J_[scale].row(i);
      A_.noalias() += H_[scale][i];
      b_.noalias() += J_[scale].row(i).transpose() * error;
      cost_sum_ += error * error;
    }
  }

}

void Dvo::readParameters(std::string config_file, int num_scales, int first_scale) {

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    int cols = fsSettings["image_width"];
    int rows = fsSettings["image_height"];
    cv::Size image_size(cols,rows);

    bm_.reset(new BlockMatching(image_size));

    cam0_topic_ = (std::string)fsSettings["cam0_topic"];
    cam1_topic_ = (std::string)fsSettings["cam1_topic"];

    cv::Mat T_cam0_imu;
    fsSettings["T_cam0_imu"] >> T_cam0_imu;
    T_c_b_ << T_cam0_imu.at<double>(0,0), T_cam0_imu.at<double>(0,1), T_cam0_imu.at<double>(0,2), T_cam0_imu.at<double>(0,3),
              T_cam0_imu.at<double>(1,0), T_cam0_imu.at<double>(1,1), T_cam0_imu.at<double>(1,2), T_cam0_imu.at<double>(1,3),
              T_cam0_imu.at<double>(2,0), T_cam0_imu.at<double>(2,1), T_cam0_imu.at<double>(2,2), T_cam0_imu.at<double>(2,3),
              T_cam0_imu.at<double>(3,0), T_cam0_imu.at<double>(3,1), T_cam0_imu.at<double>(3,2), T_cam0_imu.at<double> (3,3); 
    R_c_b_ = T_c_b_.topLeftCorner<3, 3>();
    T_b_c_ = T_c_b_.inverse();

    cv::Mat D0, k0; 
    fsSettings["cam0_distortion_coeffs"] >> D0;
    fsSettings["cam0_intrinsics"] >> k0;
    cv::Mat K0 = (cv::Mat_<double>(3,3) <<  k0.at<double>(0,0), 0, k0.at<double>(0,2),
                                          0, k0.at<double>(0,1), k0.at<double>(0,3),
                                          0, 0, 1 );

    cv::Mat D1, k1;
    fsSettings["cam1_distortion_coeffs"] >> D1;
    fsSettings["cam1_intrinsics"] >> k1;
    cv::Mat K1 = (cv::Mat_<double>(3,3) <<  k1.at<double>(0,0), 0, k1.at<double>(0,2),
                                          0, k1.at<double>(0,1), k1.at<double>(0,3),
                                          0, 0, 1 );

    cv::Mat T_cam1_cam0;
    fsSettings["T_cn_cnm1"] >> T_cam1_cam0;
    cv::Mat R = T_cam1_cam0(cv::Rect(0,0,3,3));
    cv::Mat t = T_cam1_cam0(cv::Rect(3,0,1,3));

    std::cout << "cam0_topic = " << cam0_topic_ << std::endl;
    std::cout << "cam1_topic = " << cam1_topic_ << std::endl;

    std::cout << "T_cam0_imu_ = " << T_cam0_imu << std::endl;
    std::cout << "cam0_distortion_coeffs_ = " << D0 << std::endl;
    std::cout << "cam0_intrinsics_ = " << K0 << std::endl;

    std::cout << "cam1_distortion_coeffs_ = " << D1 << std::endl;
    std::cout << "cam1_intrinsics_ = " << K1 << std::endl;

    fsSettings.release();

    // rectification init
    cv::Mat R1, P0, P1, Q;
    
    cv::stereoRectify(K0, D0, K1, D1, image_size, R, t, R0_, R1, P0, P1, Q, CV_CALIB_ZERO_DISPARITY); // TO DO: CHECK THIS FLAG
    cv::initUndistortRectifyMap(K0, D0, R0_, P0, image_size,  CV_32FC1, map0x_, map0y_);
    cv::initUndistortRectifyMap(K1, D1, R1, P1, image_size,  CV_32FC1, map1x_, map1y_);

    float fx = P0.at<double>(0,0);
    float fy = P0.at<double>(1,1);
    float cx = P0.at<double>(0,2);
    float cy = P0.at<double>(1,2);
    float baseline = cv::norm(P0(cv::Rect(3,0,1,3)) - P1(cv::Rect(3,0,1,3)))/fx;
    Eigen::Matrix3f K;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    std::cout << "K_rectified: " << K << std::endl;
    std::cout << "T_cam1_cam0: " << T_cam1_cam0 << std::endl;
    std::cout << "baseline: " << baseline << std::endl;

    K_.reserve(num_scales);
    baseline_.reserve(num_scales);
    for (int i = num_scales - 1; i >= 0; i--) { // NOTE: Lowest index means smaller image
      float scale = pow(2.0, static_cast<float>(-i - first_scale));
      baseline_.push_back(baseline / scale);
      K_.push_back(utils::scaleIntrinsics(K, scale));
    }
}

inline cv::Vec3b JetBGR(float v, float vmin, float vmax) {
  cv::Vec3b c = cv::Vec3b(255, 255, 255);

  const float dv = vmax - vmin;
  const float inv_dv = 1.0f / dv;

  v = v < vmin ? vmin : v;
  v = v > vmax ? vmax : v;

  if (v < (vmin + 0.25 * dv)) {
    c(2) = 0;
    c(1) = 4 * (v - vmin) * inv_dv * 255;
  } else if (v < (vmin + 0.5 * dv)) {
    c(2) = 0;
    c(0) = (1 + 4 * (vmin + 0.25 * dv - v) * inv_dv) * 255;
  } else if (v < (vmin + 0.75 * dv)) {
    c(2) = (4 * (v - vmin - 0.5 * dv) * inv_dv) * 255;
    c(0) = 0;
  } else {
    c(1) = (1 + 4 * (vmin + 0.75 * dv - v) * inv_dv) * 255;
    c(0) = 0;
  }

  return (c);
}

cv::Mat Dvo::visualizeDepth(cv::Mat &disparity, int scale) {
  cv::Mat colored_depth = cv::Mat(disparity.rows, disparity.cols, CV_8UC3);

  float bfx = K_[scale](0, 0) * baseline_[scale];
  disparity.forEach<float>([&](float &d, const int *position) -> void {
    colored_depth.at<cv::Vec3b>(position[0], position[1])
      = (d < 0) ? cv::Vec3b(255, 255, 255)
                : JetBGR(bfx / d, kMinDepth, kMaxDepth);
  });
  return colored_depth;
}

cv::Mat Dvo::visualizeDepthOnReference(cv::Mat &left,
                                       cv::Mat &disparity,
                                       int scale) {
  cv::Mat colored_gray;
  cv::cvtColor(left, colored_gray, cv::COLOR_GRAY2BGR);

  const float bfx = K_[scale](0, 0) * baseline_[scale];
  cv::Mat
    colored_depth_overlay = cv::Mat(disparity.rows, disparity.cols, CV_8UC3);

  disparity.forEach<float>([&](float &d, const int *position) -> void {
    int r = position[0], c = position[1];
    cv::Vec3b bg = colored_gray.at<cv::Vec3b>(r, c);
    colored_depth_overlay.at<cv::Vec3b>(r, c)
      = (d < 0) ? bg
                : 0.6 * JetBGR(bfx / d, kMinDepth, kMaxDepth) + 0.4 * bg;
  });

  return colored_depth_overlay;
}

bool Dvo::writeLocalMapToPly(const std::string &filename,
                             cv::Mat &left, cv::Mat &disparity, int scale) {
  /** happly prefers this **/
  std::vector<std::array<double, 3>> vertices, vertex_colors;

  float fx = K_[scale](0, 0);
  float fy = K_[scale](1, 1);
  float fx_inv = 1 / fx;
  float fy_inv = 1 / fy;
  float cx = K_[scale](0, 2);
  float cy = K_[scale](1, 2);
  float bfx = baseline_[scale] * fx;

  /** thread conflict will take place if we use forEach **/
  for (int v = 0; v < disparity.rows; ++v) {
    for (int u = 0; u < disparity.cols; ++u) {
      float d = disparity.at<float>(v, u);
      if (d > 0) {
        float z = bfx / d;
        float x = z * (u - cx) * fx_inv;
        float y = z * (v - cy) * fy_inv;

        std::array<double, 3> vertex = {x, y, z};
        vertices.emplace_back(vertex);

        auto c = JetBGR(z, kMinDepth, kMaxDepth);
        std::array<double, 3> vertex_color =
          {c[2] / 255.0, c[1] / 255.0, c[0] / 255.0};
        vertex_colors.emplace_back(vertex_color);
      }
    }
  }

  happly::PLYData ply_out;
  ply_out.addVertexPositions(vertices);
  ply_out.addVertexColors(vertex_colors);
  ply_out.write(filename, happly::DataFormat::Binary);
}

void Dvo::appendToGlobalMap(cv::Mat &left, cv::Mat &disparity,
                            const Eigen::Matrix4f &T,
                            float sample_rate, int scale) {
  float fx = K_[scale](0, 0);
  float fy = K_[scale](1, 1);
  float fx_inv = 1 / fx;
  float fy_inv = 1 / fy;
  float cx = K_[scale](0, 2);
  float cy = K_[scale](1, 2);
  float bfx = baseline_[scale] * fx;

  std::uniform_real_distribution<float> uni_dist(0.0f, 1.0f);
  for (int v = 0; v < disparity.rows; ++v) {
    for (int u = 0; u < disparity.cols; ++u) {
      float d = disparity.at<float>(v, u);
      if (d > 0) {
        float z = bfx / d;
        float x = z * (u - cx) * fx_inv;
        float y = z * (v - cy) * fy_inv;

        if (z < kDepthThreshold && uni_dist(rd_) < sample_rate) {
          auto vtx = Eigen::Vector4f(x, y, z, 1.0f);
          vtx.noalias() = T * vtx;
          auto vtx_h = vtx.hnormalized();
          std::array<double, 3> vertex = {vtx_h(0), vtx_h(1), vtx_h(2)};
          vertices_.emplace_back(vertex);

          float c = left.at<uchar>(v, u) / 255.0f;
          std::array<double, 3> vertex_color = {c, c, c};
          vertex_colors_.emplace_back(vertex_color);
        }
      }
    }
  }
}

bool Dvo::writeGlobalMapToPly(const std::string &filename) {
  happly::PLYData ply_out;
  ply_out.addVertexPositions(vertices_);
  ply_out.addVertexColors(vertex_colors_);
  ply_out.write(filename, happly::DataFormat::Binary);
}
