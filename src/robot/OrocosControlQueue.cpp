#include "OrocosControlQueue.h"

using namespace std;
using namespace arma;

void OrocosControlQueue::constructQueue(int argc, char** argv, int sleepTime, std::string commandTopic, std::string retPosTopic, std::string switchModeTopic, std::string retCartPosTopic,
                    std::string cartStiffnessTopic, std::string jntStiffnessTopic, std::string ptpTopic,
                    std::string commandStateTopic, std::string ptpReachedTopic, std::string addLoadTopic, ros::NodeHandle node
                ) {

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

    monComMode = -1;
    impMode = -1;
    ptpReached = 0;

    this->argc = argc;
    this->argv = argv;

    setInitValues();
    startingJoints = arma::vec(1);
    this->node = node;
    loop_rate = new ros::Rate(1.0 / sleepTime * 1e+6);

    cout << "pos topic: " << retPosTopic << endl;
    subJntPos = node.subscribe(retPosTopic, 2, &OrocosControlQueue::robotJointPosCallback, this);
    subCartPos = node.subscribe(retCartPosTopic, 2, &OrocosControlQueue::robotCartPosCallback, this);
    subComState = node.subscribe(commandStateTopic, 2, &OrocosControlQueue::commandStateCallback, this);
    subPtpReached = node.subscribe(ptpReachedTopic, 2, &OrocosControlQueue::phpReachedCallback, this);
    cout << ptpReachedTopic << endl;

    pub_set_cart_stiffness = node.advertise<iis_kukie::CartesianImpedance>(stiffnessTopic, 1);
    pub_set_joint_stiffness = node.advertise<iis_kukie::FriJointImpedance>(jntStiffnessTopic, 1);

    pubCommand = node.advertise<std_msgs::Float64MultiArray>(commandTopic, 10);
    pubSwitchMode = node.advertise<std_msgs::Int32>(switchModeTopic, 1);
    pubPtp = node.advertise<std_msgs::Float64MultiArray>(ptpTopic, 10);
//	pubAddLoad = node.advertise<std_msgs::Float32MultiArray>(addLoadTopic, 1);

    usleep(1e6);

}

OrocosControlQueue::OrocosControlQueue(int argc, char** argv, int sleepTime, string commandTopic, string retJointPosTopic, string switchModeTopic, string retCartPosTopic, string cartStiffnessTopic, string jntStiffnessTopic, string ptpTopic,
	string commandStateTopic, string ptpReachedTopic, string addLoadTopic, ros::NodeHandle node) : ControlQueue(LBR_MNJ) {

    constructQueue(argc, argv, sleepTime, commandTopic, retJointPosTopic, switchModeTopic, retCartPosTopic, cartStiffnessTopic,
                   jntStiffnessTopic, ptpTopic, commandStateTopic, ptpReachedTopic, addLoadTopic, node);

}

OrocosControlQueue::OrocosControlQueue(int argc, char** argv, int sleepTime, std::string deviceType, std::string armPrefix, ros::NodeHandle node) : ControlQueue(LBR_MNJ) {

    commandTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/move";
    retJointPosTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/get_state";
    switchModeTopic = "/" + deviceType + "/" + armPrefix + "/settings/switch_mode";
    retCartPosTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/get_pose";
    stiffnessTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/set_impedance";
    jntStiffnessTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/set_impedance";
    ptpTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/ptp";
    commandStateTopic = "/" + deviceType + "/" + armPrefix + "/settings/get_command_state";
    ptpReachedTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/ptp_reached";
    addLoadTopic = "not supported yet";

    constructQueue(argc, argv, sleepTime, commandTopic, retJointPosTopic, switchModeTopic, retCartPosTopic, stiffnessTopic,
                   jntStiffnessTopic, ptpTopic, commandStateTopic, ptpReachedTopic, addLoadTopic, node);

}

void OrocosControlQueue::robotJointPosCallback(const sensor_msgs::JointState& msg) {

	currentJointsMutex.lock();
        currentJoints = arma::vec(msg.position.size());
        for(int i = 0; i < msg.position.size(); ++i) currentJoints(i) = msg.position.at(i);
	currentJointsMutex.unlock();

}

void OrocosControlQueue::robotCartPosCallback(const geometry_msgs::Pose& msg) {

	currentCartsMutex.lock();
        currentCarts = arma::vec(6);
        currentCarts(0) = msg.position.x;
        currentCarts(1) = msg.position.y;
        currentCarts(2) = msg.position.z;
        currentCarts(3) = msg.orientation.x;
        currentCarts(4) = msg.orientation.y;
        currentCarts(5) = msg.orientation.z;
	currentCartsMutex.unlock();

}

void OrocosControlQueue::commandStateCallback(const std_msgs::Float32MultiArray& msg) {
	monComMode = msg.data[0];
	impMode = msg.data[1];
}

void OrocosControlQueue::phpReachedCallback(const std_msgs::Int32MultiArray& msg) {
	ptpReached = msg.data[0];
}

void OrocosControlQueue::run() {

	setInitValues();

    arma::vec movement = arma::vec(1);

	cout << "start moving to start position" << endl;
    if(startingJoints.n_elem > 1) moveJoints(startingJoints);
	cout << "finished moving to start position" << endl;
	
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
}

void OrocosControlQueue::setInitValues() {

	isInit = false;
	finish = 0;
	
    currentJoints = arma::vec(1);
    currentCarts = arma::vec(1);
	
	while(!movementQueue.empty()) movementQueue.pop();

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

arma::vec OrocosControlQueue::getCartesianPos() {
	return currentCarts;
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
