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
    moveit_core
    moveit_msgs
    sensor_msgs
    control_msgs
    geometry_msgs
    iis_robot_dep
    tf_conversions
    actionlib_msgs
    pcl_conversions
    message_runtime
    message_generation
    moveit_ros_planning_interface
)

set(CURSES_USE_NCURSES TRUE)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(Armadillo REQUIRED)
find_package(PythonLibs REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
    DEPENDS ${ARMADILLO_LIBRARIES} prosim gsl gslcblas ncurses stdc++ ${Boost_LIBRARIES} ${Eigen_Libraries} ${PCL_LIBRARIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    /usr/include/python3.4m
)

set(ENABLE_VISUALIZATION 1)

add_definitions(-fPIC)
add_definitions(-shared)
add_definitions(-pthread)
#add_definitions(-std=c++11)
add_definitions(-DROS_SUPPORT)
add_definitions(-DOSNAME_LINUX=1)

## python definitions
link_directories(/usr/lib/python3.4/config-3.4m-x86_64-linux-gnu)

AUX_SOURCE_DIRECTORY(src/utils/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/types/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/vision/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/trajectory/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/manipulation/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/mounted/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/robotino/ KUKADUSRC)
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

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES} ${PCL_LIBRARIES} ncurses gsl gslcblas stdc++ pthread dl util m python3.4m prosim)

catkin_add_gtest(kukadutest test/kukadutest.cpp)

add_executable(kukadu_demo_guided src/demo/guided.cpp)
target_link_libraries(kukadu_demo_guided kukadu ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(robotino_demo src/demo/robotino_demo.cpp)
target_link_libraries(robotino_demo kukadu ${PCL_LIBRARIES} ${catkin_LIBRARIES})
