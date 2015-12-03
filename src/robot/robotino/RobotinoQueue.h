#ifndef KUKADU_ROBOTINO_H
#define KUKADU_ROBOTINO_H

#include <queue>
#include <deque>
#include <math.h>
#include <time.h>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <armadillo>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <iis_robot_dep/KukieError.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <iis_robot_dep/FriRobotData.h>
#include <iis_robot_dep/FriJointState.h>
#include <std_msgs/MultiArrayDimension.h>
#include <iis_robot_dep/FriJointCommand.h>
#include <iis_robot_dep/FriRobotJntData.h>
#include <iis_robot_dep/FriJointImpedance.h>
#include <iis_robot_dep/CartesianImpedance.h>

#include "../../utils/utils.h"

#define COMMAND_NOT_SET -100

/** \brief The KukieControlQueue provides control capabilities for the Kuka LWR 4+ robotic arm
 * 
 * This class implements the abstract ControlQueue class for the usage with the iisKukie system. It provides basic functionalities such as command mode control
 * in joint space as well as point to point movement in cartesian and joint space. To use it, the additionally provided KRL script has to be selected on the robot
 * controller side. For further information how to use it, please see the sample programs and the kuka documentation
 * \ingroup RobotFramework
 */
class RobotinoQueue : public ControlQueue {

private:


public:

    RobotinoQueue();

    void run();
	void setFinish();
    void safelyDestroy();
    void setInitValues();
    void stopCurrentMode();
    void switchMode(int mode);
    void moveJoints(arma::vec joints);
    void moveJointsNb(arma::vec joints);
    void setJntPtpThresh(double thresh);
    void setStartingJoints(arma::vec joints);
    void addJointsPosToQueue(arma::vec joints);
    void moveCartesian(geometry_msgs::Pose pos);
    void moveCartesianNb(geometry_msgs::Pose pos);
    void addCartesianPosToQueue(geometry_msgs::Pose pose);
    void setAdditionalLoad(float loadMass, float loadPos);
	void synchronizeToControlQueue(int maxNumJointsInQueue);
	void setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq);

    void shutUp();
    void startTalking();

    /* roll back stops the roll back mode (recording) and kills previous experience (can only be used once per recording) */
    void rollBack(double time);
    void stopJointRollBackMode();
    void startJointRollBackMode(double possibleTime);

    bool isInitialized();

    double getTimeStep();

    std::string getRobotName();
    std::string getRobotFileName();
    std::string getRobotSidePrefix();
    std::string getRobotDeviceType();

    std::vector<std::string> getJointNames();

    mes_result getCartesianPos();
    mes_result getCurrentJoints();
    mes_result getCurrentJntFrcTrq();
    mes_result getCurrentCartesianFrcTrq();
	
    geometry_msgs::Pose getCartesianPose();
    geometry_msgs::Pose getCartesianPoseRf();
    geometry_msgs::Pose moveCartesianRelativeWf(geometry_msgs::Pose basePoseRf, geometry_msgs::Pose offset);

    arma::vec getFrcTrqCart();
    arma::vec getStartingJoints();
    arma::vec retrieveJointsFromRobot();
    
};

#endif
