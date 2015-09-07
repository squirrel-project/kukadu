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

find_package(Curses)
find_package(Boost REQUIRED)
find_package(PythonLibs 3.4 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kukadu
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
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
    /usr/include/python3.4m
    $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/
    $ENV{IIS_INCLUDE_PATH}/external_rosmake_ws/motion_control/motion_control_msgs/msg_gen/cpp/include

)

set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)

add_definitions(-O0)
add_definitions(-fPIC)
add_definitions(-shared)
add_definitions(-pthread)
add_definitions(-std=c++0x)
add_definitions(-DROS_SUPPORT)
add_definitions(-DOSNAME_LINUX=1)

## python definitions
link_directories(/usr/lib/python3.4/config-3.4m-x86_64-linux-gnu /usr/lib)

AUX_SOURCE_DIRECTORY(src/utils/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/types/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/trajectory/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/mounted/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/utils/gnuplot-cpp/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/clustering/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/manipulation/pushing/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/robotDriver/src/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/utils/easyloggingpp/src/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/metric_learning/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/robotDriver/src/kuka/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/reinforcement_learning/ KUKADUSRC)

add_subdirectory(src/learning/projective_simulation)

add_library(kukadulib
    ${KUKADUSRC}
)

find_package(Eigen REQUIRED)

add_executable(pouring src/demo/pouring.cpp)
target_link_libraries(pouring kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(book src/demo/book.cpp)
target_link_libraries(book kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(book_pick src/demo/book_pick.cpp)
target_link_libraries(book_pick kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(book_place src/demo/book_place.cpp)
target_link_libraries(book_place kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(book_lean src/demo/book_lean.cpp)
target_link_libraries(book_lean kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(testing src/testing.cpp)
target_link_libraries(testing kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a)

add_executable(icra2016 src/icra2016.cpp)
target_link_libraries(icra2016 kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a pthread dl util m python3.4m prosim)

add_executable(kukadu_screwing src/intellactScrewing.cpp)
target_link_libraries(kukadu_screwing kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(kukadu_humanoids src/humanoids2014.cpp)
target_link_libraries(kukadu_humanoids kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(kukadu_mes src/guided.cpp)
target_link_libraries(kukadu_mes kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})


add_executable(quat src/QuatDMPTest.cpp)
target_link_libraries(quat kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(coll src/collectData.cpp)
target_link_libraries(coll kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(push src/testPush.cpp)
target_link_libraries(push kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

catkin_add_gtest(kukadutest test/kukadutest.cpp)
