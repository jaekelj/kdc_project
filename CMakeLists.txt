cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
project(kdc_project)
find_package(OpenCV REQUIRED)
find_package (Eigen3 3.3 REQUIRED)
find_package(GTSAM REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  cv_bridge
  tf
  geometry_msgs
  eigen_conversions
  message_generation
  blackbird
)

add_message_files(
  FILES
  Rotations.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  blackbird
)


set(CMAKE_CXX_FLAGS "-O3 -I/usr/local/opt/llvm/include")
set(CMAKE_CXX_FLAGS "-march=native")
catkin_package(
  INCLUDE_DIRS # include
  CATKIN_DEPENDS roscpp std_msgs nav_msgs
  DEPENDS system_lib
)



# file(GLOB TARGET_SRC src/*.cpp)
set(TARGET_SRC
        src/BlockMatching.cpp
        src/imageProcessing.cpp
        src/pixelSelection.cpp
        src/weight.cpp
        src/utils.cpp
        src/Dvo.cpp
        src/MultiDvo.cpp
        src/Optimizer.cpp
        src/DynamicsFactor.cpp
        src/Parameters.cpp
)

include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} include)


add_library(kdc_project ${TARGET_SRC})
target_link_libraries(kdc_project ${OpenCV_LIBS} ${GTSAM_LIBS} ${catkin_LIBRARIES})

add_executable(vio_node src/VioNode.cpp)
target_link_libraries(vio_node
			${catkin_LIBRARIES}
			${OpenCV_LIBRARIES}
			gtsam
			${GTSAM_LIBRARIES}
			${EIGEN3_LIBRARIES}
			kdc_project)
