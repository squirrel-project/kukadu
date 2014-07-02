#ifndef ROSSCHUNK
#define ROSSCHUNK

#include <iostream>
#include <vector>
#include <string>
#include <mutex>

#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
//#include <motion_control_msgs/JointPositions.h>
#include <std_msgs/Float64MultiArray.h>
// #include <cob_srvs/Trigger.h>

#include "GenericHand.h"

#include "sdh/sdh.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"
#include "sdhoptions.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

/** \brief Provides control capabilities for the Schunk SDH robotic hand with ROS binding
 * Implements the GenericHand interface for the Schunk SDH robotic hand. Note that using this class the programm has to be executed with root rights
 * \ingroup RobotFramework
 */
class RosSchunk : public GenericHand {

private:

    SDH::cSDHBase::eGraspId currentGraspId;

    int previousCurrentPosQueueSize;

    ros::NodeHandle node;
    ros::Publisher trajPub;
    ros::Subscriber stateSub;

    std::vector<std::string> joint_names_str;

    void publishSdhJoints(std::vector<float> positions);

    std::vector<float> generateCylindricalPose(double percentage);
    std::vector<float> generateParallelPose(double percentage);
    std::vector<float> generateCentricalPose(double percentage);
    std::vector<float> generateSphericalPose(double percentage);

    bool targetReached;
    bool isFirstCallback;
    bool movementStarted;

    bool vectorsDeviate(const std::vector<double> v1, const std::vector<double> v2, double tolerance);
    std::vector<float> currentCommandedPos;
    std::vector<double> initCurrentPos;
    std::vector<double> currentPos;
    std::vector<std::vector<double>> previousCurrentPosQueue;

    std::mutex currentPosMutex;


public:

    RosSchunk(ros::NodeHandle node, std::string trajTopic, std::string stateTopic, std::string hand);

    void connectHand();
    void closeHand(double percentage, double velocity);
    void disconnectHand();
    void setGrasp(SDH::cSDHBase::eGraspId grasp);
    void safelyDestroy();

    void stateCallback(const sensor_msgs::JointState state);

};

#endif
