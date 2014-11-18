#include <cstdio>
#include <iostream>
#include <fstream>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <queue>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_poly.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <vector>

#include <RedundantKin.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include "../include/kukadu.h"
#include "../src/utils/gnuplot-cpp/gnuplot_i.hpp"

#define DOSIMULATION 1

using namespace std;
using namespace arma;

Gnuplot* g1 = NULL;

int doSimulation = DOSIMULATION;

std::shared_ptr<DictionaryGeneralizer> dmpGen = std::shared_ptr<DictionaryGeneralizer>(nullptr);
thread* switchThr = NULL;
void switchQueryPoint();
void switch2dQueryPoint();

char consoleInput = 0;
char consoleInputter();

void testIROS();
void testIROSGrasping();

double as = 1.0;
double handVelocity = 20.0;
double tolAbsErr = 1e-3;
double tolRelErr = 1e-3;

double ax = 0.1;
double az = 48.0;
double bz = (az - 1) / 4;
double tau = 0.8;

int kukaStepWaitTime = 1.8 * 1e4;
double dmpStepSize = kukaStepWaitTime * 1e-6;

ros::NodeHandle* node = NULL;

ControlQueue* raQueue = NULL;
ControlQueue* laQueue = NULL;
thread* raThr = NULL;
vec switchedTo;

int mode = -1;
string outFile;
string inFile;
string inDir;

// constant for phase stopping
double ac = 10;

string left_hardware = "left_arm";
string right_hardware = "right_arm";

string hardware = right_hardware;
	
string prefix = "real";

string moveTopic = "move";
string jntPosTopic = "get_state";
string switchTopic = "switch_mode";
string carPosTopic = "get_pose";
string setCartImpTopic = "set_impedance";
string setJntImpTopic = "set_impedance";
string jntPtpTopic = "ptp";
string getCmdStateTopic = "get_command_state";
string ptpReachedTopic = "ptp_reached";
string setAddLoadTopic = "set_additional_load";

int raPort = 49938;
int laPort = 49939;
int columns = 8;

std::vector<double> pickupTmpmys{0, 1, 2, 3, 4, 5, 6, 7, 8};
std::vector<double> catchTmpmys{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
std::vector<double> genTmpmys{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
std::vector<double> tmpsigmas{0.2, 0.8};

char* handPort = "/dev/ttyUSB1";
char* screwFile = "/home/shangl/newest.txt";

double tStart = 0.0;
double tEnd = 7.5;

int main(int argc, char** args) {

	if(argc < 2) {
		cout << "using standard values" << endl;
        inDir = "/home/c7031109/iis_robot_sw/iis_catkin_ws/src/kukadu/movements/iros2014/2d_extended_gen/";
    //    inDir = "/home/shangl/iis_robot_sw/iis_catkin_ws/src/kukadu/movements/iros2014/real_robot_2d_pickup/";
	}
	
	if(!DOSIMULATION) {
		ros::init(argc, args, "kukadu"); node = new ros::NodeHandle(); usleep(1e6);
	}
	
	
	srand((unsigned int) std::time(NULL));
	
	if(!doSimulation) {
		
		ros::init(argc, args, "kukadu");
		usleep(1e6);
		
		// execute guided measurement
		raQueue = new OrocosControlQueue(argc, args, kukaStepWaitTime,
				prefix + "/" + hardware + "/" + "joint_control" + "/" + moveTopic,
				prefix + "/" + hardware + "/" + "joint_control" + "/" + jntPosTopic,
				prefix + "/" + hardware + "/" + "settings" + "/" + switchTopic,
				prefix + "/" + hardware + "/" + "cartesian_control" + "/" + carPosTopic,
				prefix + "/" + hardware + "/" + "cartesian_control" + "/" + setCartImpTopic,
				prefix + "/" + hardware + "/" + "joint_control" + "/" + setJntImpTopic,
				prefix + "/" + hardware + "/" + "joint_control" + "/" + jntPtpTopic,
				prefix + "/" + hardware + "/" + "settings" + "/" + getCmdStateTopic,
				prefix + "/" + hardware + "/" + "sensoring" + "/" + ptpReachedTopic,
				prefix + "/" + hardware + "/" + "settings" + "/" + setAddLoadTopic,
			*node
		);
		
	}
		
    testIROS();
    // testIROSGrasping();
	getch();

	return 0;

}

// TODO: something has been changed such that it is not working anymore (maybe data files?)
void testIROSGrasping() {
	
    std::shared_ptr<ControlQueue> raQueue = std::shared_ptr<ControlQueue>(nullptr);
	QuadraticKernel* kern = new QuadraticKernel();
	
	vector<double> irosmys = {0, 1, 2, 3, 4, 5};
	vector<double> irossigmas = {0.3, 0.8};
	
	//vector<double> rlExploreSigmas = {0.5, 0.5, 0.5, 0.5};
	vector<double> rlExploreSigmas = {0.2, 0.2, 0.2, 0.2};
	int rolloutsPerUpdate = 5;
	int importanceSamplingCount = 3;
	
	columns = 8;
	
	vec trajMetricWeights(7);
	trajMetricWeights.fill(1.0);
	
	tau = 5.0;
	float tmp = 0.1;
	double ax = -log(tmp) / tau / tau;
    double relativeDistanceThresh = 1.0;
	
	cout << "tau: " << tau << endl;
	cout << "ax: " << ax << endl;

	vec newQueryPoint(2);
	newQueryPoint(0) = 8;
	newQueryPoint(1) = 8;
	
	// result for (8, 8)
	vector<double> vectorNewQueryPoint = {-0.488059, 1.37839, -2.0144, -1.87255, -0.244649, -0.120394, 1.23313};
    shared_ptr<GraspingRewardComputer> reward(new GraspingRewardComputer(vectorNewQueryPoint));
//	t_executor_res opt = reward.getOptimalTraj(5.0);
	
	cout << "execute ground truth for (8, 8)" << endl;
    t_executor_res opt = executeDemo(NULL, "/home/shangl/iis_robot_sw/iis_catkin_ws/src/kukadu/movements/iros2014/traj_8-8.txt", doSimulation, az, bz, 0);
	
	// speedup testing process by inserting already learned metric result
	mat m(2,2);
	m(0, 0) = 1.0;
	m(1, 0) = -1.3176;
	m(0, 1) = -1.3176;
	m(1, 1) = 3.2793;
//	1.0000  -1.3176
//  -1.3176   3.2793

    vec timeCenters(1);
    timeCenters(0) = 2.5;
	
//	dmpGen = new DictionaryGeneralizer(newQueryPoint, raQueue, inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, trajMetricWeights, relativeDistanceThresh, as);
    dmpGen = std::shared_ptr<DictionaryGeneralizer>(new DictionaryGeneralizer(timeCenters, newQueryPoint, raQueue, std::shared_ptr<ControlQueue>(nullptr), inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, as, m, relativeDistanceThresh, 1.0));
	
    std::vector<std::shared_ptr<Trajectory>> initTraj;
	initTraj.push_back(dmpGen->getTrajectory());
	
	cout << newQueryPoint << endl;
//	cout << "(mainScrewOrocos) first metric: " << ((LinCombDmp*) dmpGen->getTrajectory())->getMetric().getM() << endl;
	
    std::shared_ptr<LinCombDmp> lastRollout = std::shared_ptr<LinCombDmp>(nullptr);
	
    PoWER pow(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, reward, NULL, NULL, ac, dmpStepSize, tolAbsErr, tolRelErr);
	
	int plotTimes = 5;
	g1 = new Gnuplot("PoWER demo");
	int i = 0;
	
	vec initT;
	vector<vec> initY;
	
	vector<double> lastRewards;
//	while( lastRewards.size() < 3 || (lastRewards.at(0) != lastRewards.at(1) || lastRewards.at(1) != lastRewards.at(2) ) ) {
    while( i < 50 ) {

		pow.performRollout(1, 0);
        lastRollout = std::dynamic_pointer_cast<LinCombDmp>(pow.getLastUpdate());
		lastRewards.push_back(pow.getLastUpdateReward());
		if(lastRewards.size() > 3)
			lastRewards.erase(lastRewards.begin());
		
//		cout << "(mainScrewOrocos) last update metric: " << lastRollout->getMetric().getM() << endl;
//		cout << lastRollout->getMetric().getM() << endl << endl;
		
		if(i == 0) {
			initT = pow.getLastUpdateRes().t;
			initY = pow.getLastUpdateRes().y;
		}
		
		
		if( (i % 1) == 0 ) {
			
			cout << "(mainScrewOrocos) already done " << i << " iterations" << endl;
		
			int plotNum = 1;
			for(int plotTraj = 0; plotTraj < plotNum; ++plotTraj) {
				
				ostringstream convert;   // stream used for the conversion
				convert << plotTraj;
				
				string title = string("fitted sensor data (joint") + convert.str() + string(")");
				g1->set_style("lines").plot_xy(armadilloToStdVec(opt.t), armadilloToStdVec(opt.y[plotTraj]), "optimal trajectoy");
				g1->set_style("lines").plot_xy(armadilloToStdVec(initT), armadilloToStdVec(initY[plotTraj]), "initial trajectoy");
				g1->set_style("lines").plot_xy(armadilloToStdVec(pow.getLastUpdateRes().t), armadilloToStdVec(pow.getLastUpdateRes().y[plotTraj]), "generalized trajectory");
				g1->showonscreen();
				
			}
			
		}
		
		g1->reset_plot();
		
		if( (i % 20) == 19)
			g1->remove_tmpfiles();
		
		++i;
		
	}
	

	cout << "(mainScrewOrocos) execution of trajectory at new position" << endl;
	
	while( 1 ) {
	
		cout << "(mainScrewOrocos) first coordinate: " << endl;
		cin >> newQueryPoint(0);
		cout << "(mainScrewOrocos) second coordinate: " << endl;
		cin >> newQueryPoint(1);
		
		
        lastRollout = std::dynamic_pointer_cast<LinCombDmp>(dmpGen->getTrajectory());
		lastRollout->setCurrentQueryPoint(newQueryPoint);
		dmpGen->switchQueryPoint(newQueryPoint);
		
		initTraj.clear();
		initTraj.push_back(lastRollout);
		
		t_executor_res updateRes = dmpGen->simulateTrajectory();
		
		g1 = new Gnuplot("PoWER demo2");
		g1->set_style("points").plot_xy(armadilloToStdVec(updateRes.t), armadilloToStdVec(updateRes.y[0]), "generalized trajectory");
	//	g1->set_style("lines").plot_xy(armadilloToStdVec(opt.t), armadilloToStdVec(opt.y[0]), "optimal trajectory");
		g1->showonscreen();
		
		cout << "(mainScrewOrocos) press key to continue" << endl;
		getchar();
		getchar();
		
		delete g1;
		
	}
	
}

void testIROS() {
	
    std::shared_ptr<ControlQueue> raQueue = std::shared_ptr<ControlQueue>(nullptr);
	QuadraticKernel* kern = new QuadraticKernel();
	
    std::vector<double> irosmys = {0, 1, 2, 3, 4, 5, 6, 7};
	std::vector<double> irossigmas = {0.3, 0.8};
	
	//vector<double> rlExploreSigmas = {0.5, 0.5, 0.5, 0.5};
	vector<double> rlExploreSigmas = {0.2, 0.2, 0.2, 0.2};
	int rolloutsPerUpdate = 5;
	int importanceSamplingCount = 3;
	
	columns = 2;
	
	vec trajMetricWeights(1);
	trajMetricWeights.fill(1.0);
	
	tau = 5.0;
	double ax = -log((float)0.1) / tau / tau;
	double relativeDistanceThresh = 0.4;
	
	cout << "tau: " << tau << endl;
	cout << "ax: " << ax << endl;
	
	vec newQueryPoint(2);
	newQueryPoint(0) = 1.4;
	newQueryPoint(1) = 5.1;
	
//	newQueryPoint(0) = 1.85;
//	newQueryPoint(1) = 7.5;
	
//	newQueryPoint(0) = 1.2;
//	newQueryPoint(1) = 6.9;

    vec timeCenters(1);
    timeCenters(0) = 2.5;
	
    shared_ptr<GaussianObstacleRewardComputer> reward(new GaussianObstacleRewardComputer(newQueryPoint(0), 2.0, newQueryPoint(1)));
    t_executor_res opt = reward->getOptimalTraj(5.0, 0);
	
    dmpGen = std::shared_ptr<DictionaryGeneralizer>(new DictionaryGeneralizer(timeCenters, newQueryPoint, raQueue, NULL, inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, trajMetricWeights, relativeDistanceThresh, as, 1.0));
	
	/*
	switchThr = new std::thread(switchQueryPoint);
	t_executor_res genRes = dmpGen->simulateTrajectory();
	
	switchThr->join();
	*/
	
    std::vector<std::shared_ptr<Trajectory>> initTraj;
	initTraj.push_back(dmpGen->getTrajectory());
	
	cout << newQueryPoint << endl;
//	cout << "(mainScrewOrocos) first metric: " << ((LinCombDmp*) dmpGen->getTrajectory())->getMetric().getM() << endl;
	
    PoWER pow(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, reward, NULL, NULL, ac, dmpStepSize, tolAbsErr, tolRelErr);
	
	int plotTimes = 5;
	g1 = new Gnuplot("PoWER demo");
	int i = 0;
	
	vec initT;
	vector<vec> initY;
	
    std::shared_ptr<LinCombDmp> lastRollout = std::shared_ptr<LinCombDmp>(nullptr);
	vector<double> lastRewards;
	
    while( i < 200 ) {

		pow.performRollout(1, 0);
        lastRollout = std::dynamic_pointer_cast<LinCombDmp>(pow.getLastUpdate());
		lastRewards.push_back(pow.getLastUpdateReward());
		if(lastRewards.size() > 3)
			lastRewards.erase(lastRewards.begin());
		
//		cout << "(mainScrewOrocos) last update metric: " << lastRollout->getMetric().getM() << endl << endl;
		
		if(i == 0) {
			initT = pow.getLastUpdateRes().t;
			initY = pow.getLastUpdateRes().y;
		}
		
		
		if( (i % 1) == 0 ) {
			
			cout << "(mainScrewOrocos) already done " << i << " iterations" << endl;
		
			int plotNum = 1;
			for(int plotTraj = 0; plotTraj < plotNum; ++plotTraj) {
				
				ostringstream convert;   // stream used for the conversion
				convert << plotTraj;
				
				string title = string("fitted sensor data (joint") + convert.str() + string(")");
				g1->set_style("lines").plot_xy(armadilloToStdVec(opt.t), armadilloToStdVec(opt.y[plotTraj]), "optimal trajectoy");
				g1->set_style("lines").plot_xy(armadilloToStdVec(initT), armadilloToStdVec(initY[plotTraj]), "initial trajectoy");
				g1->set_style("lines").plot_xy(armadilloToStdVec(pow.getLastUpdateRes().t), armadilloToStdVec(pow.getLastUpdateRes().y[plotTraj]), "generalized trajectory");
				g1->showonscreen();
				
			}
			
		}
		
		g1->reset_plot();
		
		if( (i % 20) == 19)
			g1->remove_tmpfiles();
		
		++i;
		
	}
	
	cout << "(mainScrewOrocos) execution of trajectory at new position" << endl;
	
	while( 1 ) {
	
		cout << "(mainScrewOrocos) first coordinate: " << endl;
		cin >> newQueryPoint(0);
		cout << "(mainScrewOrocos) second coordinate: " << endl;
		cin >> newQueryPoint(1);
		
		
		lastRollout->setCurrentQueryPoint(newQueryPoint);
		dmpGen->switchQueryPoint(newQueryPoint);
		
		GaussianObstacleRewardComputer reward2(newQueryPoint(0), 2.0, newQueryPoint(1));
        t_executor_res opt2 = reward2.getOptimalTraj(5.0, 0);
		
		initTraj.clear();
		initTraj.push_back(lastRollout);
	//	PoWER pow2(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, &reward2, NULL, ac, dmpStepSize, tolAbsErr, tolRelErr);
	//	pow2.performRollout(1, 0);
	//	t_executor_res updateRes = pow2.getLastUpdateRes();
	//	lastRollout = dynamic_cast<LinCombDmp*>(pow2.getLastUpdate());
	//	cout << "(mainScrewOrocos) last reward: " << pow2.getLastUpdateReward() << endl;
		
		switchThr = new std::thread(switch2dQueryPoint);
		t_executor_res updateRes = dmpGen->simulateTrajectory();
		
		switchThr->join();
		
		GaussianObstacleRewardComputer reward3(switchedTo(0), 2.0, switchedTo(1));
        t_executor_res opt3 = reward3.getOptimalTraj(5.0, 0);
		
		g1 = new Gnuplot("PoWER demo2");
		g1->set_style("points").plot_xy(armadilloToStdVec(updateRes.t), armadilloToStdVec(updateRes.y[0]), "generalized trajectory");
		g1->set_style("lines").plot_xy(armadilloToStdVec(opt2.t), armadilloToStdVec(opt2.y[0]), "optimal trajectory 1");
		g1->set_style("lines").plot_xy(armadilloToStdVec(opt3.t), armadilloToStdVec(opt3.y[0]), "optimal trajectory 2");
		g1->showonscreen();
		
		cout << "(mainScrewOrocos) press key to continue" << endl;
		getchar();
		getchar();
		
		delete g1;
		
	}
	
	/*
	cout << "(main) plotting trajectory for query point [ ";
	for(int i = 0; i < newQueryPoint.size(); ++i) cout << newQueryPoint.at(i) << " ";
	cout << "]" << endl;

	for(int i = 0; i < dmpGen->getDegOfFreedom(); ++i) {

		ostringstream convert;   // stream used for the conversion
		convert << i;
		string title = string("fitted sensor data (joint") + convert.str() + string(")");
		g1 = new Gnuplot(title);
		for(int plotTraj = 0; plotTraj < dmpGen->getQueryPointCount(); ++plotTraj) {
				std::ostringstream o;
				o << "sample (";
				for(int k = 0; k < dmpGen->getQueryPointByIndex(plotTraj).getQueryPoint().n_elem; ++k) {
					o << dmpGen->getQueryPointByIndex(plotTraj).getQueryPoint()(k) << " ";
				}
				o << ")";
				g1->set_style("lines").plot_xy(armadilloToStdVec(dmpGen->getQueryPointByIndex(plotTraj).getDmp().getSupervisedTs()), armadilloToStdVec(dmpGen->getQueryPointByIndex(plotTraj).getDmp().getSampleYs().at(i)), o.str());
				
		}
		g1->set_style("points").plot_xy(armadilloToStdVec(genRes.t), armadilloToStdVec(genRes.y[i]), "dmp y");
		g1->showonscreen();

	}
	*/
	
}

char consoleInputter() {
	cin >> consoleInput;
	return consoleInput;
}

void switchQueryPoint() {
	
	vec qp(1);
	qp(0) = 7;
	
	sleep(5.0);
	cout << "switching execution" << endl;
	dmpGen->switchQueryPoint(qp);
	
}

void switch2dQueryPoint() {
	
	vec qp(2);
	qp(0) = 1.7;
	qp(1) = 5.0;
	
	switchedTo = qp;
	
	while(dmpGen->getCurrentTime() < 1.4) {}
	cout << "switching execution to query point " << qp.t() << endl;
	dmpGen->switchQueryPoint(qp);
	
}
