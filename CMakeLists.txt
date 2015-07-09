cmake_minimum_required(VERSION 2.8.3)
project(kukadu)

find_package(cmake_modules REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
    message_runtime
    iis_kukie
    iis_robot_dep
    iis_schunk_hardware
    planning_scene_plugin
    tf_conversions
    pcl_ros
    pcl_conversions
    sensor_msgs
)

set(CURSES_USE_NCURSES TRUE)
find_package(Curses)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

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
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/
    $ENV{IIS_INCLUDE_PATH}/external_rosmake_ws/motion_control/motion_control_msgs/msg_gen/cpp/include
)

set(CMAKE_BUILD_TYPE Debug)
add_definitions(-DOSNAME_LINUX=1)
add_definitions(-w)
add_definitions(-std=c++0x)
add_definitions(-fPIC)
add_definitions(-shared)
add_definitions(-pthread)
add_definitions(-DROS_SUPPORT)

AUX_SOURCE_DIRECTORY(src/utils/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/trajectory/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/robotDriver/src/kuka/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/robotDriver/src/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/mounted/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/utils/gnuplot-cpp/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/robot/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/types/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/utils/easyloggingpp/src/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/reinforcement_learning/ KUKADUSRC)
AUX_SOURCE_DIRECTORY(src/learning/metric_learning/ KUKADUSRC)

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

add_executable(kukadu_screwing src/intellactScrewing.cpp)
target_link_libraries(kukadu_screwing kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(kukadu_humanoids src/humanoids2014.cpp)
target_link_libraries(kukadu_humanoids kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(kukadu_mes src/guided.cpp)
target_link_libraries(kukadu_mes kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(SimInt_test src/SimInterfaceExample.cpp)
target_link_libraries(SimInt_test kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

#add_executable(DMPcart_test src/testingDMPcart.cpp)
#target_link_libraries(DMPcart_test kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

#add_executable(t_push src/teachingPush.cpp)
#target_link_libraries(t_push kukadulib ${catkin_LIBRARIES} ${Boost_LIBRARIES} $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ncurses gsl gslcblas stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})
