cmake_minimum_required(VERSION 2.8.3)

project(singlecamera_orbslam2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  message_filters
  cv_bridge
  tf
  darknet_ros_msgs
)

find_package(OpenCV  REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(THIRDPARTY_LIBRARIES
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

# Build the original source files

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Map.cc
  src/MapDrawer.cc
  src/Optimizer.cc
  src/PnPsolver.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Initializer.cc
  src/Viewer.cc
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${THIRDPARTY_LIBRARIES}
  -lboost_serialization
)

set(PROJECT_INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${PROJECT_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${THIRDPARTY_LIBRARIES}
  DEPENDS OpenCV Pangolin EIGEN3
)


###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

set(LIBRARIES
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)


# Node for monocular camera
add_executable(mono_slam_node src/ros_mono.cc)
target_link_libraries(mono_slam_node ${LIBRARIES})


# Node for RGB-D camera
add_executable(rgbd_slam_node src/ros_rgbd.cc )
add_dependencies(rgbd_slam_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(rgbd_slam_node ${LIBRARIES})

# Node for stereo camera
add_executable(stereo_slam_node src/ros_stereo.cc)
target_link_libraries(stereo_slam_node ${LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS mono_slam_node rgbd_slam_node stereo_slam_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
