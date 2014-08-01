#include "PlottingControlQueue.h"

using namespace std;
using namespace arma;

PlottingControlQueue::PlottingControlQueue(int degOfFreedom, double timeStep) : ControlQueue(degOfFreedom) {
    this->sleepTime = timeStep;
    setInitValues();
}

void PlottingControlQueue::setInitValues() {

	isInit = false;
	finish = 0;
	
	currentJoints = new float[getMovementDegreesOfFreedom()];
	currentCarts = new float[6];

}

void PlottingControlQueue::run() {

    setInitValues();
    isInit = true;

    while(!finish && ros::ok) {

    //    currentTime += sleepTime * 1e-6;
        usleep(sleepTime);

    }
}

void PlottingControlQueue::setFinish() {
	finish = 1;
	startingJoints = NULL;
}

void PlottingControlQueue::addJointsPosToQueue(float* joints) {
    currentJoints = joints;
    currentTime += sleepTime * 1e-6;
}

void PlottingControlQueue::switchMode(int mode) {

}

void PlottingControlQueue::stopCurrentMode() {
}

void PlottingControlQueue::synchronizeToControlQueue(int maxNumJointsInQueue) {
}

void PlottingControlQueue::setStartingJoints(float* joints) {
    currentJoints = joints;
    startingJoints = joints;
}

void PlottingControlQueue::moveJoints(float* joints) {
    currentJoints = joints;
}

void PlottingControlQueue::setAdditionalLoad(float loadMass, float loadPos) {
}

void PlottingControlQueue::setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) {
}

float* PlottingControlQueue::getCartesianPos() {
//	return currentCarts;
    throw "not supported yet";
}

float* PlottingControlQueue::getStartingJoints() {
	return startingJoints;
}

float* PlottingControlQueue::retrieveJointsFromRobot() {
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
