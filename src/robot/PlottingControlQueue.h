#ifndef PLOTTINGCONTROLQUEUE
#define PLOTTINGCONTROLQUEUE

#include <unistd.h>
#include <queue>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <thread>
#include <mutex>
#include <time.h>
#include <thread>

// Custom librairies
#include "ControlQueue.h"
#include "../utils/DestroyableObject.h"
#include "../utils/types.h"
#include "robotDriver/src/kuka/friRemote.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/Pose.h>
#include <iis_kukie/CartesianImpedance.h>
#include <iis_kukie/FriJointCommand.h>
#include <iis_kukie/FriJointImpedance.h>
#include <iis_kukie/FriJointState.h>
#include <iis_kukie/FriRobotData.h>
#include <iis_kukie/FriRobotJntData.h>
#include <iis_kukie/KukieError.h>

#define COMMAND_NOT_SET -100

/** \brief The OrocosControlQueue provides control capabilities for the Kuka LWR 4+ robotic arm
 * 
 * This class implements the abstract ControlQueue class for the usage with the iisorocos system. It provides basic functionalities such as command mode control
 * in joint space as well as point to point movement in cartesian and joint space. To use it, the additionally provided KRL script has to be selected on the robot
 * controller side. For further information how to use it, please see the sample programs and the kuka documentation
 * \ingroup RobotFramework
 */
class PlottingControlQueue : public ControlQueue {

private:
	
	int sleepTime;
	int finish;
	int isInit;
	int argc;
	
	int ptpReached;
	int monComMode;
	int impMode;
	int currentMode;
	
	char** argv;
	
    arma::vec startingJoints;
    arma::vec currentJoints;
    arma::vec currentCarts;
	
	double currentTime;
	
	std::mutex currentJointsMutex;
	std::mutex currentCartsMutex;
	

public:

	/** \brief Constructor for KukaControlQueue
	 * \param port port to listen for incoming fri connection
	 * \param sleepTime cycle sleep time (time between to packets sent in command mode)
	 * \param initMode control mode (e.g. command mode, monitor mode) with which the queue should be started
	 */
    PlottingControlQueue(int degOfFreedom, double timeStep);
	
	void run();
	void setFinish();
    void addJointsPosToQueue(arma::vec joints);
    void addCartesianPosToQueue(geometry_msgs::Pose pose);
	void switchMode(int mode);
	void stopCurrentMode();
	void synchronizeToControlQueue(int maxNumJointsInQueue);
    void setStartingJoints(arma::vec joints);
    void moveJoints(arma::vec joints);
    void moveCartesian(geometry_msgs::Pose pos);
    void moveCartesianNb(geometry_msgs::Pose pos);
	
	void setAdditionalLoad(float loadMass, float loadPos);
	void setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq);

    mes_result getCurrentJntFrcTrq();
    mes_result getCurrentCartesianFrcTrq();
    mes_result getCartesianPos();
    geometry_msgs::Pose getCartesianPose();

    arma::vec getStartingJoints();
    arma::vec retrieveJointsFromRobot();
	
	mes_result getCurrentJoints();
	bool isInitialized();
	void safelyDestroy();
	void setInitValues();

    std::string getRobotName();
    std::string getRobotFileName();
    std::vector<std::string> getJointNames();
    
};

#endif
