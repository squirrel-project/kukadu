#ifndef OROCOSCONTROLQUEUE
#define OROCOSCONTROLQUEUE

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
#include "../utils/utils.h"
#include "robotDriver/src/kuka/friRemote.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <iis_kukie/CartesianImpedance.h>
#include <iis_kukie/FriJointCommand.h>
#include <iis_kukie/FriJointImpedance.h>
#include <iis_kukie/FriJointState.h>
#include <iis_kukie/FriRobotData.h>
#include <iis_kukie/FriRobotJntData.h>
#include <iis_kukie/KukieError.h>
#include <RedundantKin.h>

#define COMMAND_NOT_SET -100

/** \brief The OrocosControlQueue provides control capabilities for the Kuka LWR 4+ robotic arm
 * 
 * This class implements the abstract ControlQueue class for the usage with the iisorocos system. It provides basic functionalities such as command mode control
 * in joint space as well as point to point movement in cartesian and joint space. To use it, the additionally provided KRL script has to be selected on the robot
 * controller side. For further information how to use it, please see the sample programs and the kuka documentation
 * \ingroup RobotFramework
 */
class OrocosControlQueue : public ControlQueue {

private:

    // emergency solution for now (move topic expects commands in robot frame)
    // --> moving it to an approximately align coordinate sytem and do transformations there
    double leftW2RTM[4][4];
    double leftR2WTM[4][4];
	
	int sleepTime;
	int finish;
	int isInit;
	int argc;
	
    int ptpReached;
    int cartesianPtpReached;
	int monComMode;
	int impMode;
	int currentMode;
	
	char** argv;
	
    arma::vec startingJoints;
    arma::vec currentJoints;
    arma::vec currentCarts;
    arma::vec currentJntFrqTrq;
    arma::vec currentCartFrqTrq;
	
	double currentTime;
	
    std::queue<arma::vec> movementQueue;
	std::mutex currentJointsMutex;
	std::mutex currentCartsMutex;
    std::mutex cartFrcTrqMutex;

    geometry_msgs::Pose currentCartPose;
    geometry_msgs::Pose currentCartPoseRf;
	
	std::string commandTopic;
	std::string retJointPosTopic;
	std::string retCartPosTopic;
	std::string switchModeTopic;
	std::string stiffnessTopic;
	std::string jntStiffnessTopic;
	std::string ptpTopic;
	std::string commandStateTopic;
	std::string ptpReachedTopic;
	std::string addLoadTopic;
    std::string jntFrcTrqTopic;
    std::string cartFrcTrqTopic;
    std::string cartMoveTopic;
    std::string cartPtpReachedTopic;
    std::string cartMoveQueueTopic;
    std::string cartPoseRfTopic;
    std::string jntSetPtpThreshTopic;

    std::string deviceType;
    std::string armPrefix;
	
	ros::NodeHandle node;
	ros::Rate* loop_rate;
	
	ros::Publisher pubCommand;
	ros::Publisher pubSwitchMode;
	ros::Publisher pubPtp;
    ros::Publisher pubCartPtp;
	ros::Publisher pubAddLoad;
    ros::Publisher pubCartMoveQueue;
	
	ros::Publisher pub_set_cart_stiffness;
	ros::Publisher pub_set_joint_stiffness;
    ros::Publisher pub_set_ptp_thresh;
	
	ros::Subscriber subJntPos;
	ros::Subscriber subCartPos;
	ros::Subscriber subComState;
	ros::Subscriber subPtpReached;
    ros::Subscriber subjntFrcTrq;
    ros::Subscriber subCartFrqTrq;
    ros::Subscriber subCartPtpReached;
    ros::Subscriber subCartPoseRf;
	
	double computeDistance(float* a1, float* a2, int size);

    /* orocos callback functions */
    void robotJointPosCallback(const sensor_msgs::JointState& msg);
    void robotCartPosCallback(const geometry_msgs::Pose& msg);
    void commandStateCallback(const std_msgs::Float32MultiArray& msg);
    void ptpReachedCallback(const std_msgs::Int32MultiArray& msg);
    void cartPtpReachedCallback(const std_msgs::Int32MultiArray& msg);
    void jntFrcTrqCallback(const std_msgs::Float64MultiArray& msg);
    void cartFrcTrqCallback(const geometry_msgs::Wrench& msg);
    void cartPosRfCallback(const geometry_msgs::Pose msg);

public:

    OrocosControlQueue(int argc, char** argv, int sleepTime, std::string deviceType, std::string armPrefix, ros::NodeHandle node);

    void constructQueue(int argc, char** argv, int sleepTime, std::string commandTopic, std::string retPosTopic, std::string switchModeTopic, std::string retCartPosTopic,
                        std::string cartStiffnessTopic, std::string jntStiffnessTopic, std::string ptpTopic,
                        std::string commandStateTopic, std::string ptpReachedTopic, std::string addLoadTopic, std::string jntFrcTrqTopic, std::string cartFrcTrqTopic,
                        std::string cartMoveTopic, std::string cartPtpReachedTopic, std::string cartMoveQueueTopic, std::string cartPoseRfTopic,
                        std::string setPtpThresh, ros::NodeHandle node
                    );
	
	void run();
	void setFinish();
    void addJointsPosToQueue(arma::vec joints);
	void switchMode(int mode);
	void stopCurrentMode();
	void synchronizeToControlQueue(int maxNumJointsInQueue);
    void setStartingJoints(arma::vec joints);
    void moveJoints(arma::vec joints);
    void moveCartesian(geometry_msgs::Pose pos);
    void moveCartesianNb(geometry_msgs::Pose pos);
    void setJntPtpThresh(double thresh);
    geometry_msgs::Pose moveCartesianRelativeWf(geometry_msgs::Pose basePoseRf, geometry_msgs::Pose offset);

    void addCartesianPosToQueue(geometry_msgs::Pose pose);
	
	void setAdditionalLoad(float loadMass, float loadPos);
	void setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq);

    mes_result getCurrentJntFrcTrq();
    mes_result getCurrentCartesianFrcTrq();
	
    mes_result getCartesianPos();
    geometry_msgs::Pose getCartesianPose();
    geometry_msgs::Pose getCartesianPoseRf();
    arma::vec getStartingJoints();
    arma::vec retrieveJointsFromRobot();
	
	mes_result getCurrentJoints();
	bool isInitialized();
	void safelyDestroy();
	void setInitValues();
	
    std::string getRobotName();
    std::string getRobotFileName();
    std::vector<std::string> getJointNames();

    static const int KUKA_STOP_MODE = 0;
    static const int KUKA_JNT_POS_MODE = 10;
    static const int KUKA_CART_IMP_MODE = 20;
    static const int KUKA_JNT_IMP_MODE = 30;
    
};

#endif
