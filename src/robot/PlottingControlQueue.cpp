#include "PlottingControlQueue.h"

using namespace std;
using namespace arma;

PlottingControlQueue::PlottingControlQueue(int degOfFreedom, double timeStep) : ControlQueue(degOfFreedom) {
    this->sleepTime = timeStep;
    setInitValues();
}

void PlottingControlQueue::setInitValues() {

    set_ctrlc_exit_handler();

	isInit = false;
	finish = 0;
    currentTime = 0.0;
	
    currentJoints = arma::vec(1);
    currentCarts = arma::vec(1);

}

std::string PlottingControlQueue::getRobotFileName() {
    return "simulation_plotting_control_queue";
}

std::string PlottingControlQueue::getRobotName() {
    return "Simulation (PlottingControlQueue)";
}

std::vector<std::string> PlottingControlQueue::getJointNames() {
    vector<string> ret;
    for(int i = 0; i < getMovementDegreesOfFreedom(); ++i) {
        stringstream s;
        s << i;
        ret.push_back(string("joint ") + s.str());
    }

}

void PlottingControlQueue::setJntPtpThresh(double thresh) {

}

void PlottingControlQueue::run() {

    setInitValues();
    isInit = true;

    ros::Rate rate(1 / (sleepTime * 1e-6));

    while(!finish && ros::ok) {

    //    currentTime += sleepTime * 1e-6;
        rate.sleep();

    }
}

mes_result PlottingControlQueue::getCurrentCartesianFrcTrq() {

    mes_result ret;
    vec frcTrq(6);
    frcTrq.fill(0.0);

    ret.joints = frcTrq;
    ret.time = currentTime;

    return ret;

}

mes_result PlottingControlQueue::getCurrentJntFrcTrq() {

    mes_result ret;
    vec frcTrq(getMovementDegreesOfFreedom());
    frcTrq.fill(0.0);

    ret.joints = frcTrq;
    ret.time = currentTime;

    return ret;

}

void PlottingControlQueue::setFinish() {
	finish = 1;
    startingJoints = arma::vec(1);
}

void PlottingControlQueue::addJointsPosToQueue(arma::vec joints) {
    currentJoints = joints;
    currentTime += sleepTime * 1e-6;
}

void PlottingControlQueue::addCartesianPosToQueue(geometry_msgs::Pose pose) {
    throw "not supported yet";
}

void PlottingControlQueue::switchMode(int mode) {

}

void PlottingControlQueue::stopCurrentMode() {
}

void PlottingControlQueue::synchronizeToControlQueue(int maxNumJointsInQueue) {
}

void PlottingControlQueue::setStartingJoints(arma::vec joints) {
    currentJoints = joints;
    startingJoints = joints;
}

void PlottingControlQueue::moveCartesianNb(geometry_msgs::Pose pos) {
    throw "not supported yet";
}

void PlottingControlQueue::moveCartesian(geometry_msgs::Pose pos) {
    throw "not supported yet";
}

void PlottingControlQueue::moveJoints(arma::vec joints) {
    currentJoints = joints;
}

void PlottingControlQueue::setAdditionalLoad(float loadMass, float loadPos) {
}

void PlottingControlQueue::setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) {
}

mes_result PlottingControlQueue::getCartesianPos() {
    throw "not supported yet";
}

geometry_msgs::Pose PlottingControlQueue::getCartesianPose() {
    throw "not supported yet";
}

arma::vec PlottingControlQueue::getStartingJoints() {
	return startingJoints;
}

arma::vec PlottingControlQueue::retrieveJointsFromRobot() {
	return currentJoints;
}

mes_result PlottingControlQueue::getCurrentJoints() {
	mes_result res;
	res.time = currentTime;
	res.joints = retrieveJointsFromRobot();
	return res;
}

bool PlottingControlQueue::isInitialized() {
	return isInit;
}

void PlottingControlQueue::safelyDestroy() {
}
