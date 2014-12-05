#ifndef ROSSCHUNK
#define ROSSCHUNK

#include <iostream>
#include <vector>
#include <string>
#include <mutex>
#include <limits>

#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64MultiArray.h>
#include <iis_schunk_hardware/TactileMatrix.h>
#include <iis_schunk_hardware/TactileSensor.h>

#include "GenericHand.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../../utils/utils.h"

#define SDH_IGNORE_JOINT std::numeric_limits<double>::min()

enum kukadu_grasps {eGID_CENTRICAL, eGID_CYLINDRICAL, eGID_PARALLEL, eGID_SPHERICAL};

/** \brief Provides control capabilities for the Schunk SDH robotic hand with ROS binding
 * Implements the GenericHand interface for the Schunk SDH robotic hand. Note that using this class the programm has to be executed with root rights
 * \ingroup RobotFramework
 */
class RosSchunk : public GenericHand {

private:

    kukadu_grasps currentGraspId;

    int previousCurrentPosQueueSize;

    ros::NodeHandle node;
    ros::Publisher trajPub;
    ros::Subscriber stateSub;
    ros::Subscriber tactileSub;

    std::vector<std::string> joint_names_str;
    std::vector<arma::mat> currentTactileReadings;

    std::string hand;

    std::vector<double> generateCylindricalPose(double percentage);
    std::vector<double> generateParallelPose(double percentage);
    std::vector<double> generateCentricalPose(double percentage);
    std::vector<double> generateSphericalPose(double percentage);

    bool targetReached;
    bool isFirstCallback;
    bool movementStarted;

    bool vectorsDeviate(const std::vector<double> v1, const std::vector<double> v2, double tolerance);
    std::vector<double> currentCommandedPos;
    std::vector<double> initCurrentPos;
    std::vector<double> currentPos;
    std::vector<std::vector<double>> previousCurrentPosQueue;

    std::mutex currentPosMutex;
    std::mutex tactileMutex;

    void stateCallback(const sensor_msgs::JointState& state);
    void tactileCallback(const iis_schunk_hardware::TactileSensor& state);


public:

    RosSchunk(ros::NodeHandle node, std::string type, std::string hand);

    void connectHand();
    void closeHand(double percentage, double velocity);
    void disconnectHand();
    void setGrasp(kukadu_grasps grasp);
    void safelyDestroy();

    std::vector<arma::mat> getTactileSensing();

    void publishSdhJoints(std::vector<double> positions);
    void publishSingleJoint(int idx, double pos);

    std::string getHandName();

};

#endif
