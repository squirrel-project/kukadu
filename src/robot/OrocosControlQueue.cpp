#include "OrocosControlQueue.h"
#include <tf/tf.h>

using namespace std;
using namespace arma;

void OrocosControlQueue::constructQueue(int argc, char** argv, int sleepTime, std::string commandTopic, std::string retPosTopic, std::string switchModeTopic, std::string retCartPosTopic,
                    std::string cartStiffnessTopic, std::string jntStiffnessTopic, std::string ptpTopic,
                    std::string commandStateTopic, std::string ptpReachedTopic, std::string addLoadTopic, std::string jntFrcTrqTopic, std::string cartFrcTrqTopic,
                    std::string cartMoveTopic, std::string cartPtpReachedTopic, std::string cartMoveQueueTopic, std::string cartPoseRfTopic, std::string jntSetPtpThreshTopic, ros::NodeHandle node
                ) {

    leftR2WTM[0][0] = -0.879651;
    leftR2WTM[0][1] = 0.336431;
    leftR2WTM[0][2] = 0.336196;
    leftR2WTM[0][3] = -0.492322;
    leftR2WTM[1][0] = 0.426848;
    leftR2WTM[1][1] = 0.246623;
    leftR2WTM[1][2] = 0.870044;
    leftR2WTM[1][3] = 0.754155;
    leftR2WTM[2][0] = 0.209797;
    leftR2WTM[2][1] = 0.90884;
    leftR2WTM[2][2] = -0.360547;
    leftR2WTM[2][3] = 0.629509;
    leftR2WTM[3][0] = 0.0000;
    leftR2WTM[3][1] = -0.0000;
    leftR2WTM[3][2] = -0.0000;
    leftR2WTM[3][3] = 1.0000;

    set_ctrlc_exit_handler();

    mat vecLeftR2WTM(4, 4);
    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j)
            vecLeftR2WTM(i, j) = leftR2WTM[i][j];

    mat vecLeftWT2RM = inv(vecLeftR2WTM);
    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j)
            leftW2RTM[i][j] = vecLeftWT2RM(i, j);

    currentTime = 0.0;
    this->sleepTime= sleepTime;
    this->commandTopic = commandTopic;
    this->retJointPosTopic = retPosTopic;
    this->switchModeTopic = switchModeTopic;
    this->retCartPosTopic = retCartPosTopic;
    this->stiffnessTopic = cartStiffnessTopic;
    this->jntStiffnessTopic = jntStiffnessTopic;
    this->ptpTopic = ptpTopic;
    this->commandStateTopic = commandStateTopic;
    this->ptpReachedTopic = ptpReachedTopic;
    this->addLoadTopic = addLoadTopic;
    this->jntFrcTrqTopic = jntFrcTrqTopic;
    this->cartFrcTrqTopic = cartFrcTrqTopic;
    this->cartMoveTopic = cartMoveTopic;
    this->cartPtpReachedTopic = cartPtpReachedTopic;
    this->cartMoveQueueTopic = cartMoveQueueTopic;
    this->jntSetPtpThreshTopic = jntSetPtpThreshTopic;

    monComMode = -1;
    impMode = -1;
    ptpReached = 0;
    cartesianPtpReached = 0;

    this->argc = argc;
    this->argv = argv;

    setInitValues();
    startingJoints = arma::vec(1);
    currentJntFrqTrq = arma::vec(1);
    this->node = node;
    loop_rate = new ros::Rate(1.0 / sleepTime * 1e+6);

    subJntPos = node.subscribe(retPosTopic, 2, &OrocosControlQueue::robotJointPosCallback, this);
    subCartPos = node.subscribe(retCartPosTopic, 2, &OrocosControlQueue::robotCartPosCallback, this);
    subComState = node.subscribe(commandStateTopic, 2, &OrocosControlQueue::commandStateCallback, this);
    subPtpReached = node.subscribe(ptpReachedTopic, 2, &OrocosControlQueue::ptpReachedCallback, this);
    subjntFrcTrq = node.subscribe(jntFrcTrqTopic, 2, &OrocosControlQueue::jntFrcTrqCallback, this);
    subCartFrqTrq = node.subscribe(cartFrcTrqTopic, 2, &OrocosControlQueue::cartFrcTrqCallback, this);
    subCartPtpReached = node.subscribe(cartPtpReachedTopic, 2, &OrocosControlQueue::cartPtpReachedCallback, this);
    subCartPoseRf = node.subscribe(cartPoseRfTopic, 2, &OrocosControlQueue::cartPosRfCallback, this);

    pub_set_cart_stiffness = node.advertise<iis_kukie::CartesianImpedance>(stiffnessTopic, 1);
    pub_set_joint_stiffness = node.advertise<iis_kukie::FriJointImpedance>(jntStiffnessTopic, 1);
    pubCartPtp = node.advertise<geometry_msgs::Pose>(cartMoveTopic, 1);
    pubCartMoveQueue = node.advertise<geometry_msgs::Pose>(cartMoveQueueTopic, 1);
    pub_set_ptp_thresh = node.advertise<std_msgs::Float64>(jntSetPtpThreshTopic, 1);

    pubCommand = node.advertise<std_msgs::Float64MultiArray>(commandTopic, 10);
    pubSwitchMode = node.advertise<std_msgs::Int32>(switchModeTopic, 1);
    pubPtp = node.advertise<std_msgs::Float64MultiArray>(ptpTopic, 10);
//	pubAddLoad = node.advertise<std_msgs::Float32MultiArray>(addLoadTopic, 1);

    usleep(1e6);

}

OrocosControlQueue::OrocosControlQueue(int argc, char** argv, int sleepTime, std::string deviceType, std::string armPrefix, ros::NodeHandle node) : ControlQueue(LBR_MNJ) {

    commandTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/move";
    retJointPosTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/get_state";
    switchModeTopic = "/" + deviceType + "/" + armPrefix + "/settings/switch_mode";
    retCartPosTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/get_pose_quat_wf";
    stiffnessTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/set_impedance";
    jntStiffnessTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/set_impedance";
    ptpTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/ptp";
    commandStateTopic = "/" + deviceType + "/" + armPrefix + "/settings/get_command_state";
    ptpReachedTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/ptp_reached";
    jntFrcTrqTopic = "/" + deviceType + "/" + armPrefix + "/sensoring/est_ext_jnt_trq";
    cartFrcTrqTopic = "/" + deviceType + "/" + armPrefix + "/sensoring/cartesian_wrench";
    cartMoveTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/ptpQuaternion";
    cartPtpReachedTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/ptp_reached";
    cartMoveQueueTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/move";
    cartPoseRfTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/get_pose_rf";
    jntSetPtpThreshTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/set_ptp_thresh";
    addLoadTopic = "not supported yet";

    this->deviceType = deviceType;
    this->armPrefix = armPrefix;

    constructQueue(argc, argv, sleepTime, commandTopic, retJointPosTopic, switchModeTopic, retCartPosTopic, stiffnessTopic,
                   jntStiffnessTopic, ptpTopic, commandStateTopic, ptpReachedTopic, addLoadTopic, jntFrcTrqTopic, cartFrcTrqTopic,
                   cartMoveTopic, cartPtpReachedTopic, cartMoveQueueTopic, cartPoseRfTopic, jntSetPtpThreshTopic, node);

}

void OrocosControlQueue::addCartesianPosToQueue(geometry_msgs::Pose pose) {
    pubCartMoveQueue.publish(pose);
}

void OrocosControlQueue::cartPosRfCallback(const geometry_msgs::Pose msg) {
    currentCartPoseRf = msg;
}

geometry_msgs::Pose OrocosControlQueue::getCartesianPoseRf() {
    return currentCartPoseRf;
}

void OrocosControlQueue::setJntPtpThresh(double thresh) {
    std_msgs::Float64 th;
    th.data = thresh;
    pub_set_ptp_thresh.publish(th);
}

// relative pos in worldframe
geometry_msgs::Pose OrocosControlQueue::moveCartesianRelativeWf(geometry_msgs::Pose basePoseRf, geometry_msgs::Pose offset) {

//    cout << "(OrocosControQueue) moveCartesianRelativeWf currently only consideres position and not orientation" << endl;
    double newTargetRobotPos[4] = {1, 1, 1, 1};
    double newTargetWorldPos[4] = {1, 1, 1, 1};
    newTargetRobotPos[0]  = basePoseRf.position.x;
    newTargetRobotPos[1]  = basePoseRf.position.y;
    newTargetRobotPos[2]  = basePoseRf.position.z;
    // rotate current pose to approximate world frame (hack for now)
    transformPos(newTargetWorldPos, leftR2WTM, newTargetRobotPos);

    // add relative coordinates
    newTargetWorldPos[0] = newTargetWorldPos[0] + offset.position.x;
    newTargetWorldPos[1] = newTargetWorldPos[1] + offset.position.y;
    newTargetWorldPos[2] = newTargetWorldPos[2] + offset.position.z;

    // transform back to robot fraeme
    transformPos(newTargetRobotPos, leftW2RTM, newTargetWorldPos);

    // store it back to current pose
    basePoseRf.position.x = newTargetRobotPos[0];
    basePoseRf.position.y = newTargetRobotPos[1];
    basePoseRf.position.z = newTargetRobotPos[2];

    // publish robot frame pose to move
    addCartesianPosToQueue(basePoseRf);

    return basePoseRf;

}

std::string OrocosControlQueue::getRobotFileName() {
    return string("kuka_lwr_") + deviceType + string("_") + armPrefix;
}

std::string OrocosControlQueue::getRobotName() {
    return string("KUKA LWR (") + deviceType + string(" ") + armPrefix + string(")");
}

std::vector<std::string> OrocosControlQueue::getJointNames() {
    return {"A1", "A2", "E1", "A3", "A4", "A5", "A6"};
}

void OrocosControlQueue::cartFrcTrqCallback(const geometry_msgs::Wrench& msg) {
    cartFrcTrqMutex.lock();
        currentCartFrqTrq = vec(6);
        currentCartFrqTrq(0) = msg.force.x;
        currentCartFrqTrq(1) = msg.force.y;
        currentCartFrqTrq(2) = msg.force.z;
        currentCartFrqTrq(3) = msg.torque.x;
        currentCartFrqTrq(4) = msg.torque.y;
        currentCartFrqTrq(5) = msg.torque.z;
    cartFrcTrqMutex.unlock();
}

void OrocosControlQueue::jntFrcTrqCallback(const std_msgs::Float64MultiArray& msg) {

    currentJntFrqTrq = stdToArmadilloVec(msg.data);

}

void OrocosControlQueue::robotJointPosCallback(const sensor_msgs::JointState& msg) {

	currentJointsMutex.lock();
        currentJoints = arma::vec(msg.position.size());
        for(int i = 0; i < msg.position.size(); ++i) currentJoints(i) = msg.position.at(i);
	currentJointsMutex.unlock();

}

void OrocosControlQueue::robotCartPosCallback(const geometry_msgs::Pose& msg) {

	currentCartsMutex.lock();

        currentCarts = arma::vec(7);
        currentCarts(0) = msg.position.x;
        currentCarts(1) = msg.position.y;
        currentCarts(2) = msg.position.z;
        currentCarts(3) = msg.orientation.x;
        currentCarts(4) = msg.orientation.y;
        currentCarts(5) = msg.orientation.z;
        currentCarts(6) = msg.orientation.w;

        currentCartPose = msg;

	currentCartsMutex.unlock();

}

geometry_msgs::Pose OrocosControlQueue::getCartesianPose() {
    return currentCartPose;
}

void OrocosControlQueue::commandStateCallback(const std_msgs::Float32MultiArray& msg) {
	monComMode = msg.data[0];
	impMode = msg.data[1];
}

void OrocosControlQueue::ptpReachedCallback(const std_msgs::Int32MultiArray& msg) {
    ptpReached = msg.data[0];
}

void OrocosControlQueue::cartPtpReachedCallback(const std_msgs::Int32MultiArray& msg) {
    cartesianPtpReached = msg.data[0];
}

void OrocosControlQueue::run() {

	setInitValues();

    arma::vec movement = arma::vec(1);

    if(startingJoints.n_elem > 1) {
        cout << "start moving to start position" << endl;
        moveJoints(startingJoints);
        cout << "finished moving to start position" << endl;
    }
	
	isInit = true;
	
	while(!finish && ros::ok) {

		if(movementQueue.size() > 0) {
			
			// move to position in queue
			movement = movementQueue.front();
			movementQueue.pop();
			
            /*
			motion_control_msgs::JointPositions nextCommand;
			for(int i = 0; i < getMovementDegreesOfFreedom(); ++i) {
				nextCommand.positions.push_back(movement[i]);
			}
            */

            std_msgs::Float64MultiArray nextCommand;
            for(int i = 0; i < getMovementDegreesOfFreedom(); ++i)
                nextCommand.data.push_back(movement[i]);

			pubCommand.publish(nextCommand);

		} else {

				movement = currentJoints;

		}
		currentTime += sleepTime * 1e-6;
		usleep(sleepTime);
		
		ros::spinOnce();

	}

    cout << "thread finished" << endl;
}

void OrocosControlQueue::setInitValues() {

	isInit = false;
	finish = 0;
	
    currentJoints = arma::vec(1);
    currentCarts = arma::vec(1);
	
	while(!movementQueue.empty()) movementQueue.pop();

}

mes_result OrocosControlQueue::getCurrentCartesianFrcTrq() {

    mes_result ret;

    cartFrcTrqMutex.lock();
        ret.joints = currentCartFrqTrq;
    cartFrcTrqMutex.unlock();

    ret.time = currentTime;

    return ret;

}

mes_result OrocosControlQueue::getCurrentJntFrcTrq() {

    mes_result ret;

    ret.joints = currentJntFrqTrq;
    ret.time = currentTime;

    return ret;

}

void OrocosControlQueue::setFinish() {
	finish = 1;
    startingJoints = arma::vec(1);
}

void OrocosControlQueue::addJointsPosToQueue(arma::vec joints) {
	movementQueue.push(joints);
}

void OrocosControlQueue::switchMode(int mode) {
	if(ros::ok) {
		cout << "(OrocosControlQueue) switching to mode " << mode << endl;
		std_msgs::Int32 newMode;
		newMode.data = currentMode = mode;
		if(ros::ok()) {
			pubSwitchMode.publish(newMode);
			ros::spinOnce();
		}
		while(impMode != mode) {
			loop_rate->sleep();
			ros::spinOnce();
		}
	} else {
		cout << "(OrocosControlQueue) ros error" << endl;
	}
}

void OrocosControlQueue::stopCurrentMode() {
	switchMode(0);
}

void OrocosControlQueue::synchronizeToControlQueue(int maxNumJointsInQueue) {
	while(movementQueue.size() > maxNumJointsInQueue);
}

void OrocosControlQueue::setStartingJoints(arma::vec joints) {
	startingJoints = joints;
}

void OrocosControlQueue::moveCartesianNb(geometry_msgs::Pose pos) {
    pubCartPtp.publish(pos);
}

void OrocosControlQueue::moveCartesian(geometry_msgs::Pose pos) {

    cartesianPtpReached = 0;

    if(ros::ok) {

        cout << "(OrocosControlQueue) moving" << endl;
        pubCartPtp.publish(pos);
        ros::spinOnce();

        // just to make sure, arm really reached target
        loop_rate->sleep();
        ros::spinOnce();

        while(!cartesianPtpReached) {
            loop_rate->sleep();
            loop_rate->sleep();
            loop_rate->sleep();
            ros::spinOnce();
        }
        cout << "(OrocosControlQueue) ptp movement done" << endl;

    } else {
        cout << "(OrocosControlQueue) ros error" << endl;
    }

}

void OrocosControlQueue::moveJoints(arma::vec joints) {
	
	ptpReached = 0;
	
	if(ros::ok) {
		cout << "(OrocosControlQueue) moving" << endl;
		std_msgs::Float64MultiArray newJoints;
		for(int i = 0; i < getMovementDegreesOfFreedom(); ++i) newJoints.data.push_back(joints[i]);
		newJoints.layout.dim.push_back(std_msgs::MultiArrayDimension());
		newJoints.layout.dim[0].size = getMovementDegreesOfFreedom();
		newJoints.layout.dim[0].stride = 0;
		newJoints.layout.dim[0].label = "RAD";
		pubPtp.publish(newJoints);
		ros::spinOnce();
		
		// just to make sure, arm really reached target
		ros::spinOnce();
		usleep(0.5 * 1e6);
		ros::spinOnce();
		loop_rate->sleep();
		ros::spinOnce();
		
		while(!ptpReached) {
			loop_rate->sleep();
			loop_rate->sleep();
			loop_rate->sleep();
			ros::spinOnce();
		}
		cout << "(OrocosControlQueue) ptp movement done" << endl;
		
	} else {
		cout << "(OrocosControlQueue) ros error" << endl;
	}
	
}

double OrocosControlQueue::computeDistance(float* a1, float* a2, int size) {
	double ret = 0.0;
	for(int i = 0 ; i < size; ++i) {
		ret = pow(a1[i] - a2[i], 2);
	}
	cout << "current distance: " << ret << endl;
	return ret;
}

void OrocosControlQueue::setAdditionalLoad(float loadMass, float loadPos) {

	std_msgs::Float32MultiArray msg;
	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.layout.dim[0].size = 2;
	msg.layout.dim[0].stride = 0;
	msg.layout.dim[0].label = "Load";
	msg.data.push_back(loadMass);
	msg.data.push_back(loadPos);
	
	pubAddLoad.publish(msg);
	
	int tmpMode = currentMode;
	stopCurrentMode();
	switchMode(tmpMode);

}

void OrocosControlQueue::setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) {

    iis_kukie::CartesianImpedance imp;
	
	imp.stiffness.linear.x = imp.stiffness.linear.y = imp.stiffness.linear.z = cpstiffnessxyz;
	imp.damping.linear.x = imp.damping.linear.y = imp.damping.linear.z = cpdamping;
	imp.stiffness.angular.x = imp.stiffness.angular.y = imp.stiffness.angular.z = cpstiffnessabc;
	imp.damping.angular.x = imp.damping.angular.y = imp.damping.angular.z = cpdamping;
	imp.cpmaxdelta = cpmaxdelta;
	imp.axismaxdeltatrq = axismaxdeltatrq;
	
    iis_kukie::FriJointImpedance newImpedance;
	for (int j = 0; j < 7; j++){
		newImpedance.stiffness[j] = cpstiffnessxyz;
		newImpedance.damping[j] = cpdamping;
	}

	pub_set_cart_stiffness.publish(imp);
	pub_set_joint_stiffness.publish(newImpedance);
	ros::spinOnce();

}

mes_result OrocosControlQueue::getCartesianPos() {
    mes_result ret;
    ret.joints = currentCarts;
    ret.time = currentTime;
    return ret;
}

arma::vec OrocosControlQueue::getStartingJoints() {
	return startingJoints;
}

arma::vec OrocosControlQueue::retrieveJointsFromRobot() {
	return currentJoints;
}

mes_result OrocosControlQueue::getCurrentJoints() {
	mes_result res;
	res.time = currentTime;
	res.joints = retrieveJointsFromRobot();
	return res;
}

bool OrocosControlQueue::isInitialized() {
	return isInit;
}

void OrocosControlQueue::safelyDestroy() {
}
