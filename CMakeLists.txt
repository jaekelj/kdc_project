cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)

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
        src/utils.cpp
      	src/Optimizer.cpp
)

include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} include)


add_library(kdc_project ${TARGET_SRC})
target_link_libraries(kdc_project ${OpenCV_LIBS} ${GTSAM_LIBS})

add_executable(VIO_NODE src/VioNode.cpp)
target_link_libraries(VIO_NODE 
			${catkin_LIBRARIES} 
			${OpenCV_LIBRARIES} 
			gtsam
			${GTSAM_LIBRARIES} 
			${EIGEN3_LIBRARIES} 
			kdc_project)
