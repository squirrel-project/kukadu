cmake_minimum_required(VERSION 2.8.3)
project(kukadu)

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

add_library(kukadulib src/robot/robotDriver/src/kuka/friRemote.cpp
src/robot/robotDriver/src/kuka/friUdp.cpp
src/robot/robotDriver/src/friMain.cpp
src/robot/robotDriver/src/friMisc.cpp
src/utils/utils.cpp
src/utils/conversion_utils.cpp
src/utils/Tokenizer.cpp
src/learning/GeneralFitter.cpp
src/trajectory/DMPExecutor.cpp
src/trajectory/TrajectoryGenerator.cpp
src/trajectory/DMPTrajectoryGenerator.cpp
src/trajectory/PolyTrajectoryGenerator.cpp
src/trajectory/TrajectoryDMPLearner.cpp
src/trajectory/DMPGeneralizer.cpp
src/learning/GenericKernel.cpp
src/learning/TricubeKernel.cpp
src/learning/KernelRegressor.cpp
src/learning/GaussianProcessRegressor.cpp
src/learning/LWRRegressor.cpp
src/robot/mounted/sdhoptions.cpp
src/utils/DestroyableObject.cpp
src/robot/mounted/GenericHand.cpp
src/robot/mounted/RosSchunk.cpp
src/robot/mounted/SchunkHand.cpp
src/learning/QuadraticKernel.cpp
src/learning/GaussianKernel.cpp
src/learning/reinforcement_learning/GenDMPReinforcer.cpp
src/utils/gnuplot-cpp/gnuplot_i.cpp
src/robot/ControlQueue.cpp
src/learning/reinforcement_learning/DMPReinforcer.cpp
src/learning/reinforcement_learning/TerminalCostComputer.cpp
src/robot/OrocosControlQueue.cpp
src/trajectory/DictionaryGeneralizer.cpp
src/trajectory/TrajectoryComparator.cpp
src/trajectory/DMPTrajectoryComparator.cpp
src/types/Trajectory.cpp
src/types/SingleSampleTrajectory.cpp
src/types/DMP.cpp
src/types/QueryPoint.cpp
src/types/DMPBase.cpp
src/types/LinCombDmp.cpp
src/types/DictionaryTrajectory.cpp
src/learning/metric_learning/Mahalanobis.cpp
src/learning/metric_learning/MahalanobisLearner.cpp
src/learning/metric_learning/InfTheoMetricLearner.cpp
src/learning/metric_learning/InfTheoConstraints.cpp
src/learning/metric_learning/TogersonMetricLearner.cpp
src/learning/reinforcement_learning/PoWER.cpp
src/learning/reinforcement_learning/TrajectoryBasedReward.cpp
src/learning/reinforcement_learning/SampleRewardComputer.cpp
src/learning/reinforcement_learning/GaussianObstacleRewardComputer.cpp
src/learning/reinforcement_learning/GeneralReinforcer.cpp
src/learning/reinforcement_learning/GradientDescent.cpp
src/learning/reinforcement_learning/DmpRewardComputer.cpp
src/trajectory/TrajectoryExecutor.cpp
src/robot/PlottingControlQueue.cpp
src/trajectory/TimeDepDictionaryGeneralizer.cpp
src/utils/CustomSet.cpp
)

add_executable(testing src/testing.cpp)
target_link_libraries(testing kukadulib ${catkin_LIBRARIES} gsl gslcblas SDHLibrary-CPP stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(kukadu src/mainScrewOrocos.cpp)
target_link_libraries(kukadu kukadulib ${catkin_LIBRARIES} gsl gslcblas SDHLibrary-CPP stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(kukadu_screwing src/intellactScrewing.cpp)
target_link_libraries(kukadu_screwing kukadulib ${catkin_LIBRARIES} gsl gslcblas SDHLibrary-CPP stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(kukadu_screwing2 src/intellactScrewing2.cpp)
target_link_libraries(kukadu_screwing2 kukadulib ${catkin_LIBRARIES} gsl gslcblas SDHLibrary-CPP stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(kukadu_iros src/iros2014.cpp)
target_link_libraries(kukadu_iros kukadulib ${catkin_LIBRARIES} gsl gslcblas SDHLibrary-CPP stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(kukadu_humanoids src/humanoids2014.cpp)
target_link_libraries(kukadu_humanoids kukadulib ${catkin_LIBRARIES} gsl gslcblas SDHLibrary-CPP stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})

add_executable(kukadu_mes src/guided.cpp)
target_link_libraries(kukadu_mes kukadulib ${catkin_LIBRARIES} gsl gslcblas SDHLibrary-CPP stdc++ armadillo $ENV{IIS_INCLUDE_PATH}/external_libs/RedundantKin/libRedundantKin.a ${catkin_LIBRARIES})
