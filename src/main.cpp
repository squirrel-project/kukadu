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
#include <cfloat>
#include <math.h>

#include "../include/kukadu.h"
#include "../src/utils/gnuplot-cpp/gnuplot_i.hpp"

using namespace std;
using namespace SDH;

char consoleInputter();
void got_signal(int);

char consoleInput = 0;
vector<DestroyableObject*>* destructList = NULL;

int main(int argc, char** args) {
	
	destructList = new vector<DestroyableObject*>();
	SchunkHand* hand = NULL;
	double close_ratio = 0.0;
	double handVelocity = 20.0;
	
	int columns = 8;
	int kukaPort = 49938;
	int plotNum = columns - 1;
	
	// constant for phase stopping
	double ac = 10;

	double tolAbsErr = 1e-1;
	double tolRelErr = 1e-1;

	int kukaStepWaitTime = 1.3 * 1e4;
	double dmpStepSize = kukaStepWaitTime * 1e-6;
	
	// with current implementation tStart has to be 0.0
	double tStart = 0.0;
	double tEnd = 7.5;
	double tau = 0.8;
	
	// TODO: find a better way to really determine these constants (got these values by trying out with real robot data)
//	double az = 44;
//	double bz = (az - 1) / 4;
	double az = 48.0;
	double bz = (az - 1) / 4;
	
	// vector<double> tmpmys{0, 1, 2, 3, 4, 5, 6, 7, 8};
	vector<double> tmpmys{0, 1, 2, 3, 4, 5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
	vector<double> tmpsigmas{0.2, 0.8};
	
	char* handPort = "/dev/ttyUSB1";
	
	char* file = file = "./movements/pickup-gen-1d/traj_3.txt";
	char* outFile = "./movements/traj_blub.txt";
	char* queryOutFile = "./movements/pickup-gen-1d/query_blub.txt";
	char* genBasePath = "./movements/pickup-gen-1d/";
	
	vec reinforcePoint(1);
	reinforcePoint(0) = 6.5;
	
	char c = 0;
	char mode = 0;
	while ((c = getopt (argc, args, "rmgnetsi::o::pq::b::h")) != -1) {
		 switch(c) {
			 case 'i':
				if(optarg != NULL)
					file = optarg;
				cout << "used input file: " << string(file) << endl;
				
				break;
			 case 'o':
				if(optarg != NULL)
					outFile = optarg;
				cout << "used output file: " << string(outFile) << endl;
				break;
			 case 'q':
				 if(optarg != NULL)
					 queryOutFile = optarg;
				 cout << "used query output file: " << string(queryOutFile) << endl;
				 break;
			 case 'p':
				 // not very beautiful - i know
				 char querypoint[30];
				 cout << "for which query point do you want to generalize? ";
				 cin >> querypoint;
				 reinforcePoint(0) = string_to_double(querypoint);
				 cout << "used reinforcement point: " << reinforcePoint(0) << endl;
				 break;
			 case 'b':
				 if(optarg != NULL)
					 genBasePath = optarg;
				 cout << "used gen base path: " << genBasePath << endl;
				 break;
			 default:
				mode = c;
				break;
		 }
	 }
	 optind = 1;
	
	cout << mode << endl << endl;
	std::thread* thr = NULL;
	
	Gnuplot* g1 = NULL;
	KukaControlQueue* movementQu = NULL;
	
	// exit with error
	if(argc <= 1) exit(1);
	
	
	else if(mode == 'n') {
		
		int doSimulation = 0;
		
		// important to set this to 0 such that dmp cannot slow down when deviation from expected path occurs
		ac = 0.0;
		
		// reading in file
		mat joints = readMovements(file, columns);
		
		// TODO: determine tau automatically
		double ax = -log(0.1) / joints(joints.n_rows - 1, 0) / tau;
		
		dmp_base_set baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);

		float timeCounter = 0.0;
		float* jointValues = NULL;
		float* currentJoints = NULL;

		TrajectoryDMPLearner dmpLearner(baseDef, tau, az, bz, ax, joints, columns - 1);
		t_learned_dmp learnedDmps = dmpLearner.fitTrajectories();
		vector<vec> dmpCoeffs = learnedDmps.dmpCoeffs;

		if(!doSimulation) {
			
			// Example for ControlQueue usage
			movementQu = new KukaControlQueue(kukaPort, kukaStepWaitTime, COMMAND_DEMO_COMMAND_MODE);
			destructList->push_back(movementQu);
			
			movementQu->setStiffness(150, 150, 0.7, 15000, 150, 2.0);
			
			double* tmp = createDoubleArrayFromArmaVector(learnedDmps.y0);
			float* startingJoints = new float[columns - 1];
			for(int i = 0; i < (columns - 1); ++i) startingJoints[i] = tmp[i];
			movementQu->setStartingJoints(startingJoints);
			thr = new std::thread(&KukaControlQueue::run, movementQu);

			while(!movementQu->isInitialized());
			printf("(main) robot connection initialized\n");
			currentJoints = movementQu->getCurrentJoints().joints;
			printf("(main) begin joint [%f,%f,%f,%f,%f,%f,%f]\n", currentJoints[0], currentJoints[1], currentJoints[2], currentJoints[3], currentJoints[4], currentJoints[5], currentJoints[6]);
			fflush(stdout);

		}
		
		DMPExecutor dmpexec(learnedDmps);
		t_executor_res dmpResult;
		if(doSimulation) dmpResult = dmpexec.simulateDMP(tStart, learnedDmps.tmax, dmpStepSize, tolAbsErr, tolRelErr);
		else {
			
			dmpResult = dmpexec.runDMP(ac, tStart, learnedDmps.tmax, dmpStepSize, tolAbsErr, tolRelErr, movementQu);
			movementQu->setFinish();
			thr->join();
			
		}
		
		ofstream oFile;
		oFile.open(outFile);
		
		for(int i = 0; i < dmpResult.t.size(); ++i) {
			
			double time = dmpResult.t.at(i);
			oFile << time;
			for(int j = 0; j < columns - 1; ++j) {
				oFile << "\t" << dmpResult.y[j].at(i);
			}
			oFile << endl;
		}
		
		oFile.close();

		for(int plotTraj = 0; plotTraj < plotNum; ++plotTraj) {
			
			ostringstream convert;   // stream used for the conversion
			convert << plotTraj;
			
			string title = string("fitted sensor data (joint") + convert.str() + string(")");
			g1 = new Gnuplot(title);
	//		g1->set_style("lines").plot_xy(learnedDmps.supervisedTs, learnedDmps.sampleYs[plotTraj], "sample y");
			g1->set_style("lines").plot_xy(dmpResult.t, dmpResult.y[plotTraj], "dmp y");
			g1->showonscreen();
			
		}
		
	}
	// measure trajectory
	else if(mode == 'm') {
		
		cout << "measure trajectory" << endl;
		
		movementQu = new KukaControlQueue(kukaPort, kukaStepWaitTime, COMMAND_GUIDED_MEASUREMENT);
		destructList->push_back(movementQu);
		
		thr = new std::thread(&KukaControlQueue::run, movementQu);
		while(!movementQu->isInitialized());
		
		ofstream oFile;
		oFile.open(outFile);
		
		double time = 0.0;
		double lastTime = -1.0;
		float* joints;
		
		std::thread* inputThr = NULL;
		inputThr = new std::thread(consoleInputter);
		
		while(consoleInput == 0) {
			mes_result mesRes = movementQu->getCurrentJoints();
			
			time = mesRes.time;
			joints = mesRes.joints;
			usleep(0.5 * 1e4);
			if(joints != NULL && lastTime != time) {
				oFile << time;
				for(int i = 0; i < columns - 1; ++i) oFile << "\t" << joints[i];
				oFile << endl;
				lastTime = time;
			}
		}
		
		movementQu->setFinish();
		movementQu->stopCurrentMode();
		
		oFile.close();
		
		inputThr->join();
		
		float* lastPos = movementQu->getCartesianPos();
		for(int i = 0; i < 6; ++i) cout << lastPos[i] << endl;

	// execute or simulate dmp
	} else if(mode == 'e' || mode == 's') {

		int doSimulation;
		
		if(mode == 'e') {
			cout << "execute dmp" << endl;
			doSimulation = 0;
		}
		else {
			cout << "simulate dmp" << endl;
			doSimulation = 1;
		}

		// reading in file
		mat joints = readMovements(file, columns);
		
		// TODO: determine tau automatically
		double ax = -log(0.1) / joints(joints.n_rows - 1, 0) / tau;
		
		dmp_base_set baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);

		float timeCounter = 0.0;
		float* jointValues = NULL;
		float* currentJoints = NULL;

		TrajectoryDMPLearner dmpLearner(baseDef, tau, az, bz, ax, joints, columns - 1);
		t_learned_dmp learnedDmps = dmpLearner.fitTrajectories();
		vector<vec> dmpCoeffs = learnedDmps.dmpCoeffs;

		if(!doSimulation) {
			
		//	hand = new SchunkHand("/dev/ttyUSB0");
		
		//	include again as soon as shunk hand bug is solved
		//	destructList->push_back(hand);
			
		//	hand->connectHand();
		//	hand->closeHand(0.0, handVelocity);	
			
			// Example for ControlQueue usage
			movementQu = new KukaControlQueue(kukaPort, kukaStepWaitTime, COMMAND_DEMO_COMMAND_MODE);
			movementQu->setStiffness(2000, 300, 0.7, 15000, 150, 2.0);
			destructList->push_back(movementQu);
			
			double* tmp = createDoubleArrayFromArmaVector(learnedDmps.y0);
			float* startingJoints = new float[columns - 1];
			for(int i = 0; i < (columns - 1); ++i) startingJoints[i] = tmp[i];
			movementQu->setStartingJoints(startingJoints);
			thr = new std::thread(&KukaControlQueue::run, movementQu);

			while(!movementQu->isInitialized());
			printf("(main) robot connection initialized\n");
			currentJoints = movementQu->getCurrentJoints().joints;
			printf("(main) begin joint [%f,%f,%f,%f,%f,%f,%f]\n", currentJoints[0], currentJoints[1], currentJoints[2], currentJoints[3], currentJoints[4], currentJoints[5], currentJoints[6]);
			fflush(stdout);

		}
		
		DMPExecutor dmpexec(learnedDmps);
		t_executor_res dmpResult;
		if(doSimulation) dmpResult = dmpexec.simulateDMP(tStart, learnedDmps.tmax, dmpStepSize, tolAbsErr, tolRelErr);
		else {
			
			dmpResult = dmpexec.runDMP(ac, tStart, learnedDmps.tmax, dmpStepSize, tolAbsErr, tolRelErr, movementQu);
			
			
			movementQu->setFinish();
			thr->join();
			std::cout << "(main) execution done" << endl;
			std::cout.flush();
			
		//	hand->disconnectHand();
			
			getch();
			
		}

		for(int plotTraj = 0; plotTraj < plotNum; ++plotTraj) {
			
			ostringstream convert;   // stream used for the conversion
			convert << plotTraj;
			
			string title = string("fitted sensor data (joint") + convert.str() + string(")");
			g1 = new Gnuplot(title);
			g1->set_style("lines").plot_xy(armadilloToStdVec(learnedDmps.supervisedTs), armadilloToStdVec(learnedDmps.sampleYs[plotTraj]), "sample y");
			g1->set_style("lines").plot_xy(dmpResult.t, dmpResult.y[plotTraj], "dmp y");
			g1->showonscreen();
			
		}
		
		g1 = new Gnuplot(string("internal clock plot"));
		g1->set_style("lines").plot_xy(dmpResult.t, dmpResult.internalClock, "internal clock");
		g1->showonscreen();
	/*
		if(!doSimulation) {
			movementQu->setFinish();
			thr->join();
		}
	*/
	} else if(mode == 'g') {
		
		// ax is determined automatically
		DMPGeneralizer* dmpGen = new DMPGeneralizer(genBasePath, 1, tmpmys, tmpsigmas, az, bz);
		t_querypoint qp = dmpGen->getQueryPointByIndex(1);
		
		vec newQueryPoint(1);
		newQueryPoint(0) = 13;
		TricubeKernel* kern = new TricubeKernel();
		t_learned_dmp learnedDmp = dmpGen->generalizeDmp(kern, kern, newQueryPoint, 1.0);
		
		DMPExecutor dmpexec(learnedDmp);
		t_executor_res dmpResult = dmpexec.simulateDMP(tStart, tEnd, dmpStepSize, tolAbsErr, tolRelErr);
	
		cout << "(main) plotting trajectory for query point [ ";
		for(int i = 0; i < newQueryPoint.size(); ++i) cout << newQueryPoint.at(i) << " ";
		cout << "]" << endl;

		for(int plotTraj = 0; plotTraj < 1; ++plotTraj) {

			ostringstream convert;   // stream used for the conversion
			convert << plotTraj;
			
			string title = string("fitted sensor data (joint") + convert.str() + string(")");
			g1 = new Gnuplot(title);
			g1->set_style("lines").plot_xy(dmpResult.t, dmpResult.y[plotTraj], "dmp y");
			g1->showonscreen();

		}

	} else if(mode == 'r') {
		
		cout << "(main) rl mode" << endl;
		movementQu = new KukaControlQueue(kukaPort, kukaStepWaitTime, COMMAND_DEMO_COMMAND_MODE);
		
		TricubeKernel* tricubeKernel = new TricubeKernel();
	//	GaussianKernel* gaussianKernel = new GaussianKernel(0.5, 1.0);
		GaussianKernel* gaussianKernel = new GaussianKernel(0.5, 0.05);
		DMPGeneralizer* dmpGen = new DMPGeneralizer(genBasePath, columns - 1, tmpmys, tmpsigmas, az, bz);
		
		TerminalCostComputer* cost = new TerminalCostComputer();
		GenDMPReinforcer* reinforcer = new GenDMPReinforcer(reinforcePoint, cost, dmpGen, tricubeKernel, gaussianKernel, movementQu, ac, dmpStepSize, tolAbsErr, tolRelErr);
		
		t_executor_res dmpResult;
		
		double lastCost = 0.0;
		while( reinforcer->getIsFirstIteration() || abs( lastCost = reinforcer->getLastRolloutCost() ) > 0.03) {
			cout << "(main) last cost was " << lastCost << endl;
			reinforcer->performRollout(0, 1);
			t_learned_dmp lastRollout = reinforcer->getLastRolloutParameters();
			dmpResult = reinforcer->getLastExecutionResults();
		}
		
		ofstream oFile;
		oFile.open(outFile);
		
		for(int i = 0; i < dmpResult.t.size(); ++i) {
			
			double time = dmpResult.t.at(i);
			oFile << time;
			for(int j = 0; j < columns - 1; ++j) {
				oFile << "\t" << dmpResult.y[j].at(i);
			}
			oFile << endl;
		}
		
		oFile.close();
		
		ofstream qFile;
		qFile.open(queryOutFile);
		qFile << "\t" << reinforcePoint(0) << endl;
		qFile.close();
		
	} else if(mode == 't') {
		
		
		
	/*
		// testing locallly weighted regression
	//	string lwrGenPath = "./movements/gaussian_gen_movement/";
		string lwrGenPath = "./movements/advanced_gaussian_gen_mov/";
		vector<double> testmys{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
		vector<double> testsigmas{0.2, 0.8};
		
		DMPGeneralizer* dmpGen = new DMPGeneralizer(lwrGenPath, 2, testmys, testsigmas, az, bz);

		t_querypoint qp = dmpGen->getQueryPointByIndex(1);

		vec newQueryPoint(1);
		newQueryPoint(0) = 18.0;
		
		TricubeKernel* kernel = new TricubeKernel();
		t_learned_dmp learnedDmp = dmpGen->generalizeDmp(kernel, kernel, newQueryPoint, 100);
		
		DMPExecutor dmpexec(learnedDmp);
		t_executor_res dmpResult = dmpexec.simulateDMP(tStart, 30.0, dmpStepSize, tolAbsErr, tolRelErr);
		
		cout << "(main) plotting trajectory for query point [ ";
		for(int i = 0; i < newQueryPoint.size(); ++i) cout << newQueryPoint.at(i) << " ";
		cout << "]" << endl;
		
		int samples = dmpGen->getQueryPointCount();
		t_executor_res* pickupSampleResults = new t_executor_res[samples + 1];
		for(int i = 0; i < samples; ++i) {
			DMPExecutor samplePickups(dmpGen->getQueryPointByIndex(i).dmp);
			pickupSampleResults[i] = samplePickups.simulateDMP(tStart, 30.0, dmpStepSize, tolAbsErr, tolRelErr);
		}
		pickupSampleResults[samples] = dmpResult;

		for(int plotTraj = 0; plotTraj < 1; ++plotTraj) {
				
			ostringstream convert;   // stream used for the conversion
			convert << plotTraj;
			
			string title = string("fitted sensor data (joint") + convert.str() + string(")");
			g1 = new Gnuplot(title);
			
			for(int i = 0; i < dmpGen->getQueryPointCount(); ++i) {
				
				std::ostringstream o;
				o << "sample (";
				
				for(int j = 0; j < dmpGen->getQueryPointByIndex(i).queryPoint.n_elem; ++j) {
					o << dmpGen->getQueryPointByIndex(i).queryPoint(j) << " ";
				}
				o << ")";
				g1->set_style("lines").plot_xy(*(pickupSampleResults[i].t), *(pickupSampleResults[i].y[plotTraj]), o.str());
			}
			g1->set_style("lines").plot_xy(*(pickupSampleResults[dmpGen->getQueryPointCount()].t), *(pickupSampleResults[dmpGen->getQueryPointCount()].y[plotTraj]), "simulated gen dmp");
//			g1->set_style("lines").plot_xy(*(pickupDmpResult.t), *(pickupDmpResult.y[plotTraj]), "executed gen y");
			g1->showonscreen();
				
		}
		
	*/

	
	
		// testing gaussian process regression
		vector<double>* res = testGaussianRegressor();
		string title = string("gaussian process testing");
		g1 = new Gnuplot(title);
		g1->set_style("points").plot_xy(res[0], res[1], "goal samples");
		g1->set_style("lines").plot_xy(res[2], res[3], "learned goal mapping");
		g1->showonscreen();
	
		
	/*	
		// testing generalization
		
		
		// ax is determined automatically
		DMPGeneralizer* dmpGen = new DMPGeneralizer("./movements/pickup-gen/", 7, tmpmys, tmpsigmas, az, bz);
		t_querypoint qp = dmpGen->getQueryPointByIndex(1);
		
		vec newQueryPoint(2);
		newQueryPoint(0) = 3.0;
		newQueryPoint(1) = 3.0;
		
		TricubeKernel* kernel = new TricubeKernel();
		t_learned_dmp learnedDmp = dmpGen->generalizeDmp(kernel, kernel, newQueryPoint, 10);
		
		DMPExecutor dmpexec(learnedDmp);
		t_executor_res dmpResult = dmpexec.simulateDMP(tStart, 6.5, dmpStepSize, tolAbsErr, tolRelErr);
		
		cout << "(main) plotting trajectory for query point [ ";
		for(int i = 0; i < newQueryPoint.size(); ++i) cout << newQueryPoint.at(i) << " ";
		cout << "]" << endl;

		for(int plotTraj = 0; plotTraj < 7; ++plotTraj) {

			ostringstream convert;   // stream used for the conversion
			convert << plotTraj;
			
			string title = string("fitted sensor data (joint") + convert.str() + string(")");
			g1 = new Gnuplot(title);
			g1->set_style("lines").plot_xy(*(dmpResult.t), *(dmpResult.y[plotTraj]), "dmp y");
			g1->showonscreen();

		}
	*/
	} else if(!strcmp(args[1], "-testhand")) {
		
	} else if(mode == 'h') {
		
		int handSelection = 0;
		cout << "which hand do you want to use?" << endl << "(1) left hand" << endl << "(2) right hand" << endl << "selection: ";
		cin >> handSelection;
		
		if(handSelection == 1)
			hand = new SchunkHand("/dev/ttyUSB0");
		else hand = new SchunkHand("/dev/ttyUSB1");
		hand->connectHand();
		
		int selection = 1;
		int graspSelection = 1;
		double closingPercentage = 0.0;
		cSDHBase::eGraspId currentGrasp = cSDH::eGID_CENTRICAL;
		
		while(selection != 0) {
			cout << "what do you want do do?" << endl << "(1) set grasp" << endl << "(2) open/close hand" << endl << "(0) exit" << endl << "selection: ";
			cin >> selection;
			
			switch(selection) {
				case 1:
					cout << "which grasp do you want to use?" << endl << "(1) centrical grasp" << endl << "(2) parallel grasp" << endl << "(3) cylindrical grasp" << endl << "(4) spherical grasp" << endl << "selection: ";
					cin >> graspSelection;
					
					switch(graspSelection) {
						case 1:
							currentGrasp = cSDH::eGID_CENTRICAL;
							break;
						case 2:
							currentGrasp = cSDH::eGID_PARALLEL;
							break;
						case 3:
							currentGrasp = cSDH::eGID_CYLINDRICAL;
							break;
						case 4:
							currentGrasp = cSDH::eGID_SPHERICAL;
							break;
						default:
							break;
					}
					
					hand->setGrasp(currentGrasp);
					
					break;
				case 2:
					cout << "input closing percentage...";
					cin >> closingPercentage;
					hand->closeHand(closingPercentage, 20.0);
					break;
				default:
					break;
			}
		}
		
		cout << "disconnecting hand" << endl;
		hand->disconnectHand();
		cout << "hand disconnected" << endl;
	}
	
	cout << "press a button to end program" << endl;
	getch();

	return 0;

}

char consoleInputter() {
	cin >> consoleInput;
}

void got_signal(int in) {

	for(int i = 0; i < destructList->size(); ++i) {
		DestroyableObject* destr = destructList->at(i);
		destr->safelyDestroy();
	}
	cout << "everything safely stopped" << endl;
	cout.flush();
	usleep(0.3 * 1e6);
	exit(1);

}