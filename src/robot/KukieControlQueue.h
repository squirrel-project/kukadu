#ifndef KUKIECONTROLQUEUE
#define KUKIECONTROLQUEUE

#include <unistd.h>
#include <queue>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <thread>
#include <mutex>
#include <time.h>
#include <thread>
#include <deque>

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
#include <iis_robot_dep/CartesianImpedance.h>
#include <iis_robot_dep/FriJointCommand.h>
#include <iis_robot_dep/FriJointImpedance.h>
#include <iis_robot_dep/FriJointState.h>
#include <iis_robot_dep/FriRobotData.h>
#include <iis_robot_dep/FriRobotJntData.h>
#include <iis_robot_dep/KukieError.h>

#define COMMAND_NOT_SET -100

/** \brief The KukieControlQueue provides control capabilities for the Kuka LWR 4+ robotic arm
 * 
 * This class implements the abstract ControlQueue class for the usage with the iisKukie system. It provides basic functionalities such as command mode control
 * in joint space as well as point to point movement in cartesian and joint space. To use it, the additionally provided KRL script has to be selected on the robot
 * controller side. For further information how to use it, please see the sample programs and the kuka documentation
 * \ingroup RobotFramework
 */
class KukieControlQueue : public ControlQueue {

private:
	
    int isInit;
    int finish;
    int impMode;
	int sleepTime;
    int ptpReached;
    int monComMode;
    int currentMode;
    int rollBackQueueSize;
    int cartesianPtpReached;

    bool isRealRobot;
    bool rollbackMode;

    double currentTime;
    double rollBackTime;
    double sleepTimeInSec;

    std::deque<arma::vec> rollBackQueue;

    std::queue<arma::vec> movementQueue;
    std::queue<geometry_msgs::Pose> cartesianMovementQueue;
	
    arma::vec currentCarts;
    arma::vec currentJoints;
    arma::vec startingJoints;
    arma::vec currentJntFrqTrq;
    arma::vec currentCartFrqTrq;

    std::mutex cartFrcTrqMutex;
    std::mutex currentCartsMutex;
	std::mutex currentJointsMutex;

    geometry_msgs::Pose currentCartPose;
    geometry_msgs::Pose currentCartPoseRf;
	

    std::string ptpTopic;
    std::string armPrefix;
    std::string deviceType;
	std::string commandTopic;
    std::string addLoadTopic;
    std::string cartPtpTopic;
    std::string stiffnessTopic;
    std::string jntFrcTrqTopic;
    std::string cartFrcTrqTopic;
    std::string retCartPosTopic;
    std::string switchModeTopic;
    std::string ptpReachedTopic;
    std::string cartPoseRfTopic;
	std::string retJointPosTopic;
	std::string jntStiffnessTopic;
	std::string commandStateTopic;
    std::string cartPtpReachedTopic;
    std::string cartMoveRfQueueTopic;
    std::string cartMoveWfQueueTopic;
    std::string jntSetPtpThreshTopic;

	ros::NodeHandle node;

	ros::Rate* loop_rate;
	
    ros::Publisher pubPtp;
	ros::Publisher pubCommand;
    ros::Publisher pubCartPtp;
    ros::Publisher pubAddLoad;
	ros::Publisher pubSwitchMode;
    ros::Publisher pubCartMoveRfQueue;
    ros::Publisher pubCartMoveWfQueue;
    ros::Publisher pub_set_ptp_thresh;
	ros::Publisher pub_set_cart_stiffness;
	ros::Publisher pub_set_joint_stiffness;
	
	ros::Subscriber subJntPos;
	ros::Subscriber subCartPos;
	ros::Subscriber subComState;
    ros::Subscriber subjntFrcTrq;
	ros::Subscriber subPtpReached;
    ros::Subscriber subCartFrqTrq;
    ros::Subscriber subCartPoseRf;
    ros::Subscriber subCartPtpReached;

    /* Kukie callback functions */
    void cartPosRfCallback(const geometry_msgs::Pose msg);
    void robotCartPosCallback(const geometry_msgs::Pose& msg);
    void cartFrcTrqCallback(const geometry_msgs::Wrench& msg);
    void jntMoveCallback(const std_msgs::Float64MultiArray& msg);
    void ptpReachedCallback(const std_msgs::Int32MultiArray& msg);
    void robotJointPosCallback(const sensor_msgs::JointState& msg);
    void jntFrcTrqCallback(const std_msgs::Float64MultiArray& msg);
    void commandStateCallback(const std_msgs::Float32MultiArray& msg);
    void cartPtpReachedCallback(const std_msgs::Int32MultiArray& msg);

    double computeDistance(float* a1, float* a2, int size);

public:

    KukieControlQueue(int sleepTime, std::string deviceType, std::string armPrefix, ros::NodeHandle node);

    void constructQueue(int sleepTime, std::string commandTopic, std::string retPosTopic, std::string switchModeTopic, std::string retCartPosTopic,
                        std::string cartStiffnessTopic, std::string jntStiffnessTopic, std::string ptpTopic,
                        std::string commandStateTopic, std::string ptpReachedTopic, std::string addLoadTopic, std::string jntFrcTrqTopic, std::string cartFrcTrqTopic,
                        std::string cartPtpTopic, std::string cartPtpReachedTopic, std::string cartMoveRfQueueTopic, std::string cartMoveWfQueueTopic, std::string cartPoseRfTopic,
                        std::string setPtpThresh, ros::NodeHandle node
                    );
	
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

    static const int KUKA_STOP_MODE = 0;
    static const int KUKA_JNT_POS_MODE = 10;
    static const int KUKA_CART_IMP_MODE = 20;
    static const int KUKA_JNT_IMP_MODE = 30;

    static const int KUKA_STD_XYZ_STIFF = 250;
    static const int KUKA_STD_ABC_STIFF = 100;
    static const int KUKA_STD_CPDAMPING = 0.3;
    
};

#endif
