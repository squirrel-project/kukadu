#include "KukaControlQueue.h"

#include <unistd.h>
#include <queue>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <stdio.h>

#define CONTROLQUEUE_DEBUG 0

using namespace std;

KukaControlQueue::KukaControlQueue(int port, int sleepTime, int initMode) : ControlQueue(LBR_MNJ) {

	this->port = port;
	this->sleepTime = sleepTime;
	this->initMode = initMode;

	friInst = NULL;
	currentJoints = NULL;
	isInit = false;
	finish = 0;

	currentMode = COMMAND_NOT_SET;
	loadMass = 0.0;
	loadPos = 0.0;
	
	setInitValues();
	
	initializeRobot(port);

}

void KukaControlQueue::setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) {
	
	stiffnessMutex.lock();
		
		this->cpstiffnessxyz = cpstiffnessxyz;
		this->cpstiffnessabc = cpstiffnessabc;
		this->cpdamping = cpdamping;
		this->cpmaxdelta = cpmaxdelta;
		this->maxforce = maxforce;
		this->axismaxdeltatrq = axismaxdeltatrq;
		
	stiffnessMutex.unlock();
	
}

void KukaControlQueue::setAdditionalLoad(float loadMass, float loadPos) {
	this->loadMass = loadMass;
	this->loadPos = loadPos;
}

void KukaControlQueue::setInitValues() {

	isInit = false;
	finish = 0;
	
	stiffnessMutex.lock();
	
		cpstiffnessxyz = 2000;
		cpstiffnessabc = 150;
		cpdamping = 0.7;
		cpmaxdelta = 150;
		maxforce = 150;
		axismaxdeltatrq = 2.0;
	
	stiffnessMutex.unlock();

	currentMode = COMMAND_NOT_SET;
	
	while(!movementQueue.empty()) movementQueue.pop();

}

void KukaControlQueue::safelyDestroy() {
	this->closeRobotSession();
}

void KukaControlQueue::synchronizeToControlQueue(int maxNumJointsInQueue) {
	while(movementQueue.size() > maxNumJointsInQueue);
}

void KukaControlQueue::setStartingJoints(float* joints) {
	this->startingJoints = joints;
}

void KukaControlQueue::run() {
	
	float* movement = NULL;
	float* previousMovement = NULL;
	float* measuredJoints = NULL;
	
	int stopLoop = 0;
	
	cout << "start moving to start position" << endl;
	if(startingJoints != NULL) moveJoints(startingJoints);
	
	cout << "finished moving to start position" << endl;
	
	if(initMode == COMMAND_DEMO_COMMAND_MODE) previousMovement = setupCommandMode(port);
	else if(initMode == COMMAND_GUIDED_MEASUREMENT) previousMovement = setupGuidedMeasurementMode(port);
	
	currentJoints = new float[getMovementDegreesOfFreedom()];
	for(int i = 0; i < getMovementDegreesOfFreedom(); ++i) currentJoints[i] = previousMovement[i];
	
	timeThreadStarted = 0.0;
	currentTime = 0.0;
	
	while(!finish) {

		usleep(sleepTime);
		if(movementQueue.size() > 0) {
			
			// move to position in queue
			movement = movementQueue.front();
			movementQueue.pop();

			// TODO get rid of the memory leak
			previousMovement = movement;

		} else {

			// send dummy packet (stay where you are)
			if(previousMovement != movement)
				movement = previousMovement;

		}

		// send packet where robot should move
		float* tmpJoints = currentJoints;
		float* tmpJoints2;
		
		if(currentMode == COMMAND_DEMO_COMMAND_MODE) tmpJoints2 = controlRobot(movement);
		else if(currentMode == COMMAND_GUIDED_MEASUREMENT)  tmpJoints2 = retrieveJointsFromRobot();
		
		// ensure thread safety
		currentJointsMutex.lock();
			currentTime += sleepTime * 1e-6;
			currentJoints = tmpJoints2;
		currentJointsMutex.unlock();
		
		delete(tmpJoints);

	}
	
	closeRobotSession();
	setInitValues();

}

float* KukaControlQueue::getStartingJoints() {
	return startingJoints;
}

void KukaControlQueue::moveJoints(float* joints) {
	
	// Reset the command
	friInst->setToKRLInt(0, COMMAND_NONE);
	friInst->doDataExchange();
	
	// Send the command
	friInst->setToKRLInt(0, COMMAND_MOVE_JOINTS);
	
	for (int i = 0; i < getMovementDegreesOfFreedom(); i++) // Copy the command data
		friInst->setToKRLReal(i, (float) joints[i]);
	
	friInst->doDataExchange();

	// Wait for the controller to acknowledge the command (busy flag set)
	while (!friInst->getFrmKRLBool(0))
	friInst->doDataExchange();

	// Reset the command
	friInst->setToKRLInt(0, COMMAND_NONE);
	friInst->doDataExchange();

	// Loop while the controller is busy
	while (friInst->getFrmKRLBool(0)) {
		friInst->doDataExchange();
	}
	
}

void KukaControlQueue::setFinish() {
	finish = 1;
}

bool KukaControlQueue::isInitialized() {
	return isInit;
}

mes_result KukaControlQueue::getCurrentJoints() {
	
	mes_result ret;
	float* retJoint = NULL;
	
	if(currentJoints != NULL) {
		retJoint = new float[getMovementDegreesOfFreedom()];
		// ensure thread safety
		currentJointsMutex.lock();
		for(int i = 0; i < getMovementDegreesOfFreedom(); ++i) {
			retJoint[i] = currentJoints[i];
		}
		ret.time = currentTime;
		currentJointsMutex.unlock();
	}
	
	ret.joints = retJoint;
	
	return ret;

}
float* KukaControlQueue::getCartesianPos() {
	
	float* currentPositionValues = new float[6];
	
	switchMode(COMMAND_GET_POSITION);
	
	while(friInst->getFrmKRLBool(0))
		friInst->doDataExchange();

	for (int i = 0; i < 6; i++)
		currentPositionValues[i] = friInst->getFrmKRLReal(i);
	
	return currentPositionValues;
	
}

float* KukaControlQueue::retrieveJointsFromRobot() {

	friInst->doDataExchange(); // Exchange packets
	float* currentJointValues = new float[getMovementDegreesOfFreedom()];
	// Copy the values
	float* tmp = friInst->getMsrMsrJntPosition();
	for (int i = 0; i < getMovementDegreesOfFreedom(); i++)
		currentJointValues[i] = tmp[i];
	return currentJointValues;

}

void KukaControlQueue::addJointsPosToQueue(float* joints) {
	movementQueue.push(joints);
	if(CONTROLQUEUE_DEBUG) {
		printf("added joint. new queue size is %d\n", movementQueue.size());
		fflush(stdout);
	}
}

void KukaControlQueue::switchMode(int mode) {
	
	if(currentMode != COMMAND_NOT_SET) stopCurrentMode();
	// Send the command
	friInst->setToKRLInt(0, mode);
	friInst->doDataExchange();

	// Wait for the controller to acknowledge the command (busy flag set)
	while (!friInst->getFrmKRLBool(0)) {
		friInst->doDataExchange();
	}
	
	// Reset the command
	friInst->setToKRLInt(0, COMMAND_NONE);
	friInst->doDataExchange();
	
	currentMode = mode;
	
}

void KukaControlQueue::stopCurrentMode() {
	
	// Send the command
	friInst->setToKRLInt(0, COMMAND_STOP);
	friInst->doDataExchange();
	
	// Wait for the controller to acknowledge the command (busy flag set)
	while (friInst->getFrmKRLBool(0))
		friInst->doDataExchange();
	
}

float* KukaControlQueue::setupGuidedMeasurementMode(int listenningPort) {
	
	switchMode(COMMAND_GUIDED_MEASUREMENT);
	isInit = true;
	return retrieveJointsFromRobot();
}

float* KukaControlQueue::setupCommandMode(int listenningPort) {
	
	float* ret = NULL;
	FRI_QUALITY lastQuality = FRI_QUALITY_UNACCEPTABLE;
	
	// submit additional mass data to kuka (measured in kg) ($fri_frm_rea[11])
	friInst->setToKRLReal(9, loadMass);
	
	// submit mass offset to kuka (measured in mm)
	friInst->setToKRLReal(10, loadPos);
	
	// setup stiffness
	friInst->setToKRLReal(8, cpstiffnessabc);
	friInst->setToKRLReal(11, cpstiffnessxyz);
	friInst->setToKRLReal(12, cpdamping);
	friInst->setToKRLReal(13, cpmaxdelta);
	friInst->setToKRLReal(14, maxforce);
	friInst->setToKRLReal(15, axismaxdeltatrq);
	
	friInst->doDataExchange();
	
	switchMode(COMMAND_DEMO_COMMAND_MODE);

	// Wait to be in 'command' mode
	ret = new float[getMovementDegreesOfFreedom()];
	while (1) {
		float* tmp = friInst->getMsrCmdJntPosition();
		for (int i = 0; i < getMovementDegreesOfFreedom(); i++)
			ret[i] = tmp[i]; // Get current joint values

		friInst->doPositionControl(ret); // Exchange packets
		if (friInst->getQuality() != lastQuality) { // Connection quality just changed
			printf("(ControlQueue) Connection quality changed (%d -> %d)\n", lastQuality, friInst->getQuality());
			lastQuality = friInst->getQuality();
		}
		
		tmp = friInst->getMsrCmdJntPosition();
		for (int i = 0; i < getMovementDegreesOfFreedom(); i++)
			ret[i] = tmp[i]; // Get current joint values

		// Check conditions for breaking the loop
		if (friInst->getState() == FRI_STATE_CMD && friInst->isPowerOn()) // We are in 'command' mode
			break; // Go on
		if (!friInst->getFrmKRLBool(0)) // Something's wrong on the controller side: it's not busy anymore
			exit(0); // Stop
	}
	
	cout << "(ControlQueue) Switched to command mode" << endl;
	currentMode = COMMAND_DEMO_COMMAND_MODE;
	isInit = true;
	
	return ret;
	
}

void KukaControlQueue::initializeRobot(int listenningPort) {
	
	// Initializations
	fri_checkSetup();
	friInst = new friRemote(listenningPort);
	
}

void KukaControlQueue::closeRobotSession() {
	stopCurrentMode();
}

float* KukaControlQueue::controlRobot(float* newJoint) {
	
	if(CONTROLQUEUE_DEBUG) {
		printf("size of queue %d\t", movementQueue.size());
		printf("total joints %d\t", getMovementDegreesOfFreedom());
		printf("goto joint [%f,%f,%f,%f,%f,%f,%f]\n", newJoint[0], newJoint[1], newJoint[2], newJoint[3], newJoint[4], newJoint[5], newJoint[6]);
		fflush(stdout);
	}
	
	float* jointValues = new float[getMovementDegreesOfFreedom()];
	
	friInst->doJntImpedanceControl(newJoint);
//	friInst->doPositionControl(newJoint);
	
	float* tmp = friInst->getMsrMsrJntPosition();
	for(int i = 0; i < getMovementDegreesOfFreedom(); ++i) jointValues[i] = tmp[i];

	// Check conditions for breaking the loop
	if (!friInst->getFrmKRLBool(0)) // Something's wrong on the controller side: it's not busy anymore
		exit(0);
	if (friInst->getState() != FRI_STATE_CMD) // We are not in command mode anymore
		exit(0);
	
	return jointValues;
	
}