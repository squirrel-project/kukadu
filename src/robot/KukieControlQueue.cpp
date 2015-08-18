#include "KukieControlQueue.h"
#include <tf/tf.h>

using namespace std;
using namespace arma;

void KukieControlQueue::constructQueue(int sleepTime, std::string commandTopic, std::string retPosTopic, std::string switchModeTopic, std::string retCartPosTopic,
                    std::string cartStiffnessTopic, std::string jntStiffnessTopic, std::string ptpTopic,
                    std::string commandStateTopic, std::string ptpReachedTopic, std::string addLoadTopic, std::string jntFrcTrqTopic, std::string cartFrcTrqTopic,
                    std::string cartPtpTopic, std::string cartPtpReachedTopic, std::string cartMoveRfQueueTopic, std::string cartMoveWfQueueTopic, std::string cartPoseRfTopic, std::string jntSetPtpThreshTopic, ros::NodeHandle node
                ) {

    set_ctrlc_exit_handler();

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
    this->cartPtpTopic = cartPtpTopic;
    this->cartPtpReachedTopic = cartPtpReachedTopic;
    this->cartMoveRfQueueTopic = cartMoveRfQueueTopic;
    this->cartMoveWfQueueTopic = cartMoveWfQueueTopic;
    this->jntSetPtpThreshTopic = jntSetPtpThreshTopic;

    monComMode = -1;
    impMode = -1;
    ptpReached = 0;
    cartesianPtpReached = 0;

    setInitValues();
    startingJoints = arma::vec(1);
    currentJntFrqTrq = arma::vec(1);
    this->node = node;
    loop_rate = new ros::Rate(1.0 / sleepTime * 1e+6);

    subJntPos = node.subscribe(retPosTopic, 2, &KukieControlQueue::robotJointPosCallback, this);
    subCartPos = node.subscribe(retCartPosTopic, 2, &KukieControlQueue::robotCartPosCallback, this);
    subComState = node.subscribe(commandStateTopic, 2, &KukieControlQueue::commandStateCallback, this);
    subPtpReached = node.subscribe(ptpReachedTopic, 2, &KukieControlQueue::ptpReachedCallback, this);
    subjntFrcTrq = node.subscribe(jntFrcTrqTopic, 2, &KukieControlQueue::jntFrcTrqCallback, this);
    subCartFrqTrq = node.subscribe(cartFrcTrqTopic, 2, &KukieControlQueue::cartFrcTrqCallback, this);
    subCartPtpReached = node.subscribe(cartPtpReachedTopic, 2, &KukieControlQueue::cartPtpReachedCallback, this);
    subCartPoseRf = node.subscribe(cartPoseRfTopic, 2, &KukieControlQueue::cartPosRfCallback, this);

    pub_set_cart_stiffness = node.advertise<iis_robot_dep::CartesianImpedance>(stiffnessTopic, 1);
    pub_set_joint_stiffness = node.advertise<iis_robot_dep::FriJointImpedance>(jntStiffnessTopic, 1);
    pubCartPtp = node.advertise<geometry_msgs::Pose>(cartPtpTopic, 1);
    pubCartMoveRfQueue = node.advertise<geometry_msgs::Pose>(cartMoveRfQueueTopic, 1);
    pubCartMoveWfQueue = node.advertise<geometry_msgs::Pose>(cartMoveWfQueueTopic, 1);
    pub_set_ptp_thresh = node.advertise<std_msgs::Float64>(jntSetPtpThreshTopic, 1);

    pubCommand = node.advertise<std_msgs::Float64MultiArray>(commandTopic, 10);
    pubSwitchMode = node.advertise<std_msgs::Int32>(switchModeTopic, 1);
    pubPtp = node.advertise<std_msgs::Float64MultiArray>(ptpTopic, 10);
//	pubAddLoad = node.advertise<std_msgs::Float32MultiArray>(addLoadTopic, 1);

    usleep(1e6);

}

KukieControlQueue::KukieControlQueue(int sleepTime, std::string deviceType, std::string armPrefix, ros::NodeHandle node) : ControlQueue(LBR_MNJ) {

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
    cartPtpTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/ptpQuaternion";
    cartPtpReachedTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/ptp_reached";
    cartMoveRfQueueTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/move_rf";
    cartMoveWfQueueTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/move_wf";
    cartPoseRfTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/get_pose_quat_rf";
    jntSetPtpThreshTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/set_ptp_thresh";
    addLoadTopic = "not supported yet";

    this->deviceType = deviceType;
    this->armPrefix = armPrefix;

    constructQueue(sleepTime, commandTopic, retJointPosTopic, switchModeTopic, retCartPosTopic, stiffnessTopic,
                   jntStiffnessTopic, ptpTopic, commandStateTopic, ptpReachedTopic, addLoadTopic, jntFrcTrqTopic, cartFrcTrqTopic,
                   cartPtpTopic, cartPtpReachedTopic, cartMoveRfQueueTopic, cartMoveWfQueueTopic, cartPoseRfTopic, jntSetPtpThreshTopic, node);

}

double KukieControlQueue::getTimeStep() {
    return sleepTime * 1e-6;
}

void KukieControlQueue::addCartesianPosToQueue(geometry_msgs::Pose pose) {
    cartesianMovementQueue.push(pose);
}

void KukieControlQueue::cartPosRfCallback(const geometry_msgs::Pose msg) {
    currentCartPoseRf = msg;
}

geometry_msgs::Pose KukieControlQueue::getCartesianPoseRf() {
    return currentCartPoseRf;
}

void KukieControlQueue::setJntPtpThresh(double thresh) {
    std_msgs::Float64 th;
    th.data = thresh;
    pub_set_ptp_thresh.publish(th);
}

// relative pos in worldframe
geometry_msgs::Pose KukieControlQueue::moveCartesianRelativeWf(geometry_msgs::Pose basePoseRf, geometry_msgs::Pose offset) {

    double newTargetWorldPos[4] = {1, 1, 1, 1};

    // add relative coordinates
    newTargetWorldPos[0] = basePoseRf.position.x + offset.position.x;
    newTargetWorldPos[1] = basePoseRf.position.y + offset.position.y;
    newTargetWorldPos[2] = basePoseRf.position.z + offset.position.z;

    // store it back to current pose
    basePoseRf.position.x = newTargetWorldPos[0];
    basePoseRf.position.y = newTargetWorldPos[1];
    basePoseRf.position.z = newTargetWorldPos[2];

    // publish robot frame pose to move
    addCartesianPosToQueue(basePoseRf);

    return basePoseRf;

}

std::string KukieControlQueue::getRobotFileName() {
    return string("kuka_lwr_") + deviceType + string("_") + armPrefix;
}

std::string KukieControlQueue::getRobotName() {
    return string("KUKA LWR (") + deviceType + string(" ") + armPrefix + string(")");
}

std::vector<std::string> KukieControlQueue::getJointNames() {
    return {"A1", "A2", "E1", "A3", "A4", "A5", "A6"};
}

void KukieControlQueue::cartFrcTrqCallback(const geometry_msgs::Wrench& msg) {
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

void KukieControlQueue::jntFrcTrqCallback(const std_msgs::Float64MultiArray& msg) {

    currentJntFrqTrq = stdToArmadilloVec(msg.data);

}

void KukieControlQueue::robotJointPosCallback(const sensor_msgs::JointState& msg) {

	currentJointsMutex.lock();
        currentJoints = arma::vec(msg.position.size());
        for(int i = 0; i < msg.position.size(); ++i) currentJoints(i) = msg.position.at(i);
	currentJointsMutex.unlock();

}

void KukieControlQueue::robotCartPosCallback(const geometry_msgs::Pose& msg) {

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

geometry_msgs::Pose KukieControlQueue::getCartesianPose() {
    return currentCartPose;
}

void KukieControlQueue::commandStateCallback(const std_msgs::Float32MultiArray& msg) {
	monComMode = msg.data[0];
	impMode = msg.data[1];
}

void KukieControlQueue::ptpReachedCallback(const std_msgs::Int32MultiArray& msg) {
    ptpReached = msg.data[0];
}

void KukieControlQueue::cartPtpReachedCallback(const std_msgs::Int32MultiArray& msg) {
    cartesianPtpReached = msg.data[0];
}

void KukieControlQueue::run() {

    setInitValues();
    ros::Rate sleepRate(1e6 / sleepTime);

    arma::vec movement = arma::vec(1);
    geometry_msgs::Pose movementPose;

    if(startingJoints.n_elem > 1) {
        cout << "start moving to start position" << endl;
        moveJoints(startingJoints);
        cout << "finished moving to start position" << endl;
    }
	
	isInit = true;
	
	while(!finish && ros::ok) {

        if(currentMode == KUKA_JNT_IMP_MODE || currentMode == KUKA_JNT_POS_MODE) {

            if(movementQueue.size() > 0) {

                // move to position in queue
                movement = movementQueue.front();
                movementQueue.pop();

                std_msgs::Float64MultiArray nextCommand;
                for(int i = 0; i < getMovementDegreesOfFreedom(); ++i)
                    nextCommand.data.push_back(movement[i]);

                pubCommand.publish(nextCommand);

            } else {

                movement = currentJoints;

            }

        } else if(currentMode == KUKA_CART_IMP_MODE) {

            if(cartesianMovementQueue.size() > 0) {

                // move to position in queue
                movementPose = cartesianMovementQueue.front();
                cartesianMovementQueue.pop();

                pubCartMoveWfQueue.publish(movementPose);

            } else {

                movementPose = currentCartPose;

            }

        }

		currentTime += sleepTime * 1e-6;
        sleepRate.sleep();
		
        ros::spinOnce();

	}

    cout << "thread finished" << endl;
}

void KukieControlQueue::setInitValues() {

	isInit = false;
	finish = 0;
	
    currentJoints = arma::vec(1);
    currentCarts = arma::vec(1);

    while(!movementQueue.empty()) movementQueue.pop();

}

mes_result KukieControlQueue::getCurrentCartesianFrcTrq() {

    mes_result ret;

    cartFrcTrqMutex.lock();
        ret.joints = currentCartFrqTrq;
    cartFrcTrqMutex.unlock();

    ret.time = currentTime;

    return ret;

}

mes_result KukieControlQueue::getCurrentJntFrcTrq() {

    mes_result ret;

    ret.joints = currentJntFrqTrq;
    ret.time = currentTime;

    return ret;

}

void KukieControlQueue::setFinish() {
	finish = 1;
    startingJoints = arma::vec(1);
}

void KukieControlQueue::addJointsPosToQueue(arma::vec joints) {
    movementQueue.push(joints);
}

void KukieControlQueue::switchMode(int mode) {
	if(ros::ok) {
        cout << "(KukieControlQueue) switching to mode " << mode << endl;
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
        cout << "(KukieControlQueue) ros error" << endl;
	}
}

void KukieControlQueue::stopCurrentMode() {
    switchMode(KUKA_STOP_MODE);
    switchMode(KUKA_JNT_POS_MODE);
    switchMode(KUKA_STOP_MODE);
}

void KukieControlQueue::synchronizeToControlQueue(int maxNumJointsInQueue) {
    if(currentMode == KUKA_JNT_IMP_MODE || currentMode == KUKA_JNT_POS_MODE) {
        while(movementQueue.size() > maxNumJointsInQueue);
    } else if(currentMode == KUKA_CART_IMP_MODE) {
        while(cartesianMovementQueue.size() > maxNumJointsInQueue);
    }
}

void KukieControlQueue::setStartingJoints(arma::vec joints) {
	startingJoints = joints;
}

void KukieControlQueue::moveCartesianNb(geometry_msgs::Pose pos) {
    pubCartPtp.publish(pos);
}

void KukieControlQueue::moveCartesian(geometry_msgs::Pose pos) {

    cartesianPtpReached = 0;

    if(ros::ok) {

        cout << "(KukieControlQueue) moving" << endl;
        pubCartPtp.publish(pos);
        ros::spinOnce();

        // just to make sure, arm really reached target
        loop_rate->sleep();
        ros::spinOnce();

        double start=ros::Time().toSec();

        while(!cartesianPtpReached) {
            loop_rate->sleep();
            loop_rate->sleep();
            loop_rate->sleep();
            ros::spinOnce();
            if (ros::Time().toSec()-start > 30.0){
                throw new std::string("(KukieControlQueue) time limit reached; ptp movement not done ");
            }
        }
        cout << "(KukieControlQueue) ptp movement done" << endl;

    } else {
        cout << "(KukieControlQueue) ros error" << endl;
    }

}

void KukieControlQueue::moveJoints(arma::vec joints) {
	
	ptpReached = 0;
	
	if(ros::ok) {
        cout << "(KukieControlQueue) moving" << endl;
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
        cout << "(KukieControlQueue) ptp movement done" << endl;
		
	} else {
        cout << "(KukieControlQueue) ros error" << endl;
	}
	
}

double KukieControlQueue::computeDistance(float* a1, float* a2, int size) {
	double ret = 0.0;
	for(int i = 0 ; i < size; ++i) {
		ret = pow(a1[i] - a2[i], 2);
	}
	cout << "current distance: " << ret << endl;
	return ret;
}

void KukieControlQueue::setAdditionalLoad(float loadMass, float loadPos) {

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

void KukieControlQueue::setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) {

    iis_robot_dep::CartesianImpedance imp;
	
	imp.stiffness.linear.x = imp.stiffness.linear.y = imp.stiffness.linear.z = cpstiffnessxyz;
	imp.damping.linear.x = imp.damping.linear.y = imp.damping.linear.z = cpdamping;
	imp.stiffness.angular.x = imp.stiffness.angular.y = imp.stiffness.angular.z = cpstiffnessabc;
	imp.damping.angular.x = imp.damping.angular.y = imp.damping.angular.z = cpdamping;
	imp.cpmaxdelta = cpmaxdelta;
	imp.axismaxdeltatrq = axismaxdeltatrq;
	
    iis_robot_dep::FriJointImpedance newImpedance;
	for (int j = 0; j < 7; j++){
		newImpedance.stiffness[j] = cpstiffnessxyz;
		newImpedance.damping[j] = cpdamping;
	}

	pub_set_cart_stiffness.publish(imp);
	pub_set_joint_stiffness.publish(newImpedance);
	ros::spinOnce();

}

mes_result KukieControlQueue::getCartesianPos() {
    mes_result ret;
    ret.joints = currentCarts;
    ret.time = currentTime;
    return ret;
}

arma::vec KukieControlQueue::getStartingJoints() {
	return startingJoints;
}

arma::vec KukieControlQueue::retrieveJointsFromRobot() {
	return currentJoints;
}

mes_result KukieControlQueue::getCurrentJoints() {
	mes_result res;
	res.time = currentTime;
	res.joints = retrieveJointsFromRobot();
	return res;
}

bool KukieControlQueue::isInitialized() {
	return isInit;
}

void KukieControlQueue::safelyDestroy() {
}

/*int KukieControlQueue::getMode(){

    return currentMode;
}*/
