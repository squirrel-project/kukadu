cmake_minimum_required(VERSION 2.8.3)
project(kukadu)

find_package(cmake_modules REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    pcl_ros
    std_msgs
    iis_kukie
    sensor_msgs
    vrep_common
    geometry_msgs
    iis_robot_dep
    tf_conversions
    pcl_conversions
    message_runtime
    message_generation
    iis_schunk_hardware
    planning_scene_plugin
)

set(CURSES_USE_NCURSES TRUE)
find_package(PCL REQUIRED)
find_package(Boost COMPONENTS filesystem program_options)
find_package(Eigen REQUIRED)
find_package(Curses REQUIRED)
find_package(PythonLibs REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
    DEPENDS armadillo prosim gsl gslcblas ncurses $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a stdc++ ${Boost_LIBRARIES} ${Eigen_Libraries} ${PCL_LIBRARIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    /usr/include/python3.4m
    $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/
    $ENV{IIS_INCLUDE_PATH}/external_rosmake_ws/motion_control/motion_control_msgs/msg_gen/cpp/include
    ${EIGEN3_INCLUDE_DIR}
)

set(ENABLE_VISUALIZATION 1)

## do not switch to debug --> pcl tools will not work anymore due to pcl bug
set(CMAKE_BUILD_TYPE Release)

#add_definitions(-O0)
add_definitions(-fPIC)
add_definitions(-shared)
add_definitions(-pthread)
add_definitions(-DUSEBOOST)
#add_definitions(-std=c++11)
add_definitions(-DROS_SUPPORT)
add_definitions(-DOSNAME_LINUX=1)

## python definitions
link_directories(/usr/lib/python3.4/config-3.4m-x86_64-linux-gnu /usr/lib)

AUX_SOURCE_DIRECTORY(src/utils/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/types/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/vision/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/trajectory/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/manipulation/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/mounted/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/utils/gnuplot-cpp/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/clustering/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/manipulation/haptic/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/manipulation/pushing/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/robotDriver/src/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/utils/easyloggingpp/src/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/metric_learning/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/robotDriver/src/kuka/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/reinforcement_learning/ KUKADUSRC)

add_subdirectory(src/learning/projective_simulation)

add_library(${PROJECT_NAME}
    ${KUKADUSRC}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a pthread dl util m python3.4m prosim ${PCL_LIBRARIES})

catkin_add_gtest(kukadutest test/kukadutest.cpp)
