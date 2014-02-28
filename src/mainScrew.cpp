#include <stdio.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <queue>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_poly.h>
#include <signal.h>

#include "../include/kukadu.h"

int main(int argc, char** args) {
	
	int raPort = 49938;
	int laPort = 49939;
	int columns = 8;
	int kukaStepWaitTime = 1.8 * 1e4;
	double dmpStepSize = kukaStepWaitTime * 1e-6;
	
	char* handPort = "/dev/ttyUSB1";
	char* pickupFile = "./movements/two_arms/leftpickup.txt";
	char* catchFile = "./movements/two_arms/catch.txt";
	
	double tStart = 0.0;
	double az = 15.0;
	double bz = (az - 1.0) / 4.0;
	double ac = 5.0;
	double handVelocity = 20.0;
	double tolAbsErr = 1e-3;
	double tolRelErr = 1e-3;
	
	vector<double> pickupTmpmys{0, 1, 2, 3, 4, 5, 6, 7, 8};
	vector<double> catchTmpmys{0, 1, 2, 3, 4, 5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
	vector<double> tmpsigmas{0.2, 0.8};
	
	KukaControlQueue* raQueue = NULL;
	thread* raThr = NULL;
	
	SchunkHand* raHand = NULL;
	raHand = new SchunkHand(handPort);
	
	raQueue = new KukaControlQueue(raPort, kukaStepWaitTime, COMMAND_DEMO_COMMAND_MODE);
	
	raHand->connectHand();
	raHand->closeHand(0.0, handVelocity);
	
	TrajectoryDMPLearner pickupLearner(pickupTmpmys, tmpsigmas, az, bz, pickupFile, columns - 1);
	t_learned_dmp pickDmps = pickupLearner.fitTrajectories();
	
	TrajectoryDMPLearner catchLearner(catchTmpmys, tmpsigmas, az, bz, catchFile, columns - 1);
	t_learned_dmp catchDmps = catchLearner.fitTrajectories();
	
	DMPExecutor pickupDmpexec(pickDmps);
	t_executor_res pickupDmpResult;
	
	DMPExecutor catchDmpexec(catchDmps);
	t_executor_res catchDmpResult;
	
	double* tmp = createDoubleArrayFromArmaVector(pickDmps.y0);
	float* startingJoints = new float[columns - 1];
	for(int i = 0; i < (columns - 1); ++i) startingJoints[i] = tmp[i];
	raQueue->setStartingJoints(startingJoints);
	raThr = raQueue->startQueueThread();
	
	pickupDmpResult = pickupDmpexec.runDMP(ac, tStart, pickDmps.tmax, dmpStepSize, tolAbsErr, tolRelErr, raQueue);
	raHand->closeHand(1.0, handVelocity);
	
	raQueue->setFinish();
	raThr->join();
	
	raQueue->setAdditionalLoad(0.15, 135);
	raQueue->moveJoints(startingJoints);
	
	tmp = createDoubleArrayFromArmaVector(catchDmps.y0);
	startingJoints = new float[columns - 1];
	for(int i = 0; i < (columns - 1); ++i) startingJoints[i] = tmp[i];
	raQueue->setStartingJoints(startingJoints);
	raThr = raQueue->startQueueThread();
/*
//	catchDmpResult = catchDmpexec.runDMP(ac, tStart, catchDmps.tmax, dmpStepSize, tolAbsErr, tolRelErr, raQueue);
	
	raQueue->setFinish();
	raThr->join();
	
//	raHand->closeHand(0.0, handVelocity);
	
//	raHand->disconnectHand();
*/

	catchDmpResult = catchDmpexec.runDMP(ac, tStart, catchDmps.tmax, dmpStepSize, tolAbsErr, tolRelErr, raQueue);
	
	raQueue->setFinish();
	raThr->join();
	
	raHand->closeHand(0.0, handVelocity);
	
	raHand->disconnectHand();

	return 0;
	
}