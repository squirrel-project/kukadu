cmake_minimum_required(VERSION 2.8.3)
project(kukadu)

find_package(cmake_modules REQUIRED)

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
set(Python_ADDITIONAL_VERSIONS 3.4)
find_package(PythonLibs 3.4 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(Armadillo REQUIRED)
find_package(PythonLibs REQUIRED)

catkin_package(
    INCLUDE_DIRS include external/ReflexxesTypeII/include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
    DEPENDS prosim gsl gslcblas ncurses stdc++ ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES}
)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    external/ReflexxesTypeII/include
)

set(ENABLE_VISUALIZATION 1)

add_definitions(-fPIC)
add_definitions(-shared)
add_definitions(-pthread)
add_definitions(-DROS_SUPPORT)
add_definitions(-DOSNAME_LINUX=1)

AUX_SOURCE_DIRECTORY(src/utils/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/types/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/vision/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/trajectory/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/manipulation/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/mounted/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/kinematics/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/utils/gnuplot-cpp/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/clustering/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/manipulation/haptic/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/manipulation/pushing/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/robotDriver/src/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/utils/easyloggingpp/src/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/metric_learning/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/robotDriver/src/kuka/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/reinforcement_learning/ KUKADUSRC)

AUX_SOURCE_DIRECTORY(external/ReflexxesTypeII/src/TypeIIRML/ REFLEXXESSRC)

add_subdirectory(src/learning/projective_simulation)

add_library(${PROJECT_NAME} ${KUKADUSRC})
add_library(reflexxeslib ${REFLEXXESSRC})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${ARMADILLO_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} reflexxeslib ncurses gsl gslcblas stdc++ pthread dl util m prosim)

catkin_add_gtest(kukadutest test/kukadutest.cpp)

add_executable(kukadu_demo_guided src/demo/guided.cpp)
target_link_libraries(kukadu_demo_guided kukadu ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(robotino_demo src/demo/robotino_demo.cpp)
target_link_libraries(robotino_demo kukadu ${PCL_LIBRARIES} ${catkin_LIBRARIES})
