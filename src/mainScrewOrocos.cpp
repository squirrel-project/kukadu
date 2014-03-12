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

#define DOSIMULATION 0

using namespace std;
using namespace arma;

Gnuplot* g1 = NULL;

int doSimulation = DOSIMULATION;

DictionaryGeneralizer* dmpGen = NULL;
thread* switchThr = NULL;
void switchQueryPoint();
void switch2dQueryPoint();

char consoleInput = 0;
char consoleInputter();

void testIROS();
void testIROSGrasping();

void testPoWER();
void testMetric();
void testDictionaryGen();
void testTrajectoryMetric();

double as = 1.0;
double az = 48.0;
double bz = (az - 1) / 4;
double handVelocity = 20.0;
double tolAbsErr = 1e-3;
double tolRelErr = 1e-3;

float pickStiffnessxyz = 1500;
float pickStiffnessabc = 500;
float pickDamping = 1.0;
float pickMaxDelta = 99;
float pickMaxForce = 150;
float pickMaxAxisTorque = 2.0;

double ax = 0.1;
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
// char* screwFile = "/home/shangl/catkin_ws/src/kukadu/src/kukadu_core/movements/orocos_demo/screw3.txt";
// char* screwFile = "/home/shangl/leftscrew.txt";
char* screwFile = "/home/shangl/newest.txt";
// char* screwFile = "/home/shangl/blub.txt";


// with current implementation tStart has to be 0.0
double tStart = 0.0;
double tEnd = 7.5;

int main(int argc, char** args) {

    if(argc < 2) {
        cout << "to less arguments - will use standard arguments" << endl;
    //    return 1;
    }
	
	if(!DOSIMULATION) {
		ros::init(argc, args, "kukadu"); node = new ros::NodeHandle(); usleep(1e6);
	}
	
	if(!strcmp(args[1], "screw"))
		mode = 0;
	else if(!strcmp(args[1], "mes")) {
		// sample call: rosrun kukadu kukadu mes /home/shangl/demomes.txt
		// sample call: rosrun kukadu kukadu mes /home/shangl/iis_robot_sw/iis_catkin_ws/src/kukadu/src/kukadu_core/movements/iros2014/real_robot_2d_pickup/traj_8-6.txt
		mode = 1;

        if(!strcmp(args[2], "right") || !strcmp(args[2], "left"))
            hardware = string(args[2]) + "_arm";
        else
            throw "(mainScrewOrocos) arm not defined";

        outFile = args[3];
        cout << "outfile: " << outFile << endl;
	}
	else if(!strcmp(args[1], "demo")) {
		// sample call: rosrun kukadu kukadu demo /home/shangl/iis_robot_sw/iis_catkin_ws/src/kukadu/src/kukadu_core/movements/thesis_sample/pickup-gen-1d/traj_5.txt
		mode = 2;
		inFile = args[2];
	}
	else if(!strcmp(args[1], "gen")) {
		// sample call: rosrun kukadu kukadu gen /home/shangl/iis_robot_sw/iis_catkin_ws/src/kukadu/src/kukadu_core/movements/pickup-1d/
		mode = 3;
		inDir = args[2];
	}
	else if(!strcmp(args[1], "guided")) {
		// sample call: rosrun kukadu kukadu guided /home/shangl/iis_robot_sw/iis_catkin_ws/src/kukadu/src/kukadu_core/movements/thesis_sample/pickup-gen-1d/traj_5.txt
		// sample call: rosrun kukadu kukadu guided /home/shangl/iis_robot_sw/iis_catkin_ws/src/kukadu/src/kukadu_core/movements/iros2014/real_robot_2d_pickup/traj_8-6.txt /home/shangl/iis_robot_sw/iis_catkin_ws/src/kukadu/src/kukadu_core/movements/iros2014/real_robot_2d_pickup/traj_8-7.txt
		mode = 4;
		inFile = args[2];
		outFile = args[3];
	}
	else if(!strcmp(args[1], "grasps")) {
		// sample call: rosrun kukadu kukadu grasp /home/shangl/grasps.txt
		mode = 5;
		outFile = args[2];
	}
	else if(!strcmp(args[1], "testing")) {
		// sample call: rosrun kukadu kukadu testing
		mode = 6;
	} else if(!strcmp(args[1], "iros")) {
		// sample call: rosrun kukadu kukadu iros /home/shangl/iis_robot_sw/iis_catkin_ws/src/kukadu/src/kukadu_core/movements/iros2014/2d_extended_gen/
		// sample call: rosrun kukadu kukadu iros /home/shangl/iis_robot_sw/iis_catkin_ws/src/kukadu/src/kukadu_core/movements/iros2014/real_robot_2d_pickup/
		mode = 7;
        if(argc < 2)
            inDir = "/home/shangl/iis_robot_sw/iis_catkin_ws/src/kukadu/src/kukadu_core/movements/iros2014/2d_extended_gen/";
        else
            inDir = args[2];
	} else {
		cerr << "mode not supported" << endl;
		exit(1);
	}
	
	// TODO: find a better way to really determine these constants (got these values by trying out with real robot data)
//	double az = 44;
//	double bz = (az - 1) / 4;
	
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
	
	if(mode == 0 && !doSimulation) {
		
		// execute screwing
		
		SchunkHand* raHand = NULL;
		raHand = new SchunkHand(handPort);
		
		string prefix = "Left";
		raQueue = new OrocosControlQueue(argc, args, kukaStepWaitTime, prefix + string("MoveJoints"), prefix + string("PosJoints"), prefix + string("SwitchMode"),
						prefix + string("PosCartesian"), prefix + string("SetCartImpedance"), prefix + string("SetJointImpedance"),
						string("/joint_control/") + prefix + ("_arm/JointPtp"), prefix + string("GetCommandState"), string("/joint_control/") + prefix + ("_arm/PtpReached"), prefix + string("SetAdditionalLoad"), *node);
		
		prefix = "Right";
		laQueue = new OrocosControlQueue(argc, args, kukaStepWaitTime, prefix + string("MoveJoints"), prefix + string("PosJoints"), prefix + string("SwitchMode"),
						prefix + string("PosCartesian"), prefix + string("SetCartImpedance"), prefix + string("SetJointImpedance"),
						string("/joint_control/") + prefix + ("_arm/JointPtp"), prefix + string("GetCommandState"), string("/joint_control/") + prefix + ("_arm/PtpReached"), prefix + string("SetAdditionalLoad"), *node);
		
		/*
		float initJoints[7] = {-0.5295850038528442, 0.6593417525291443, -0.06263750791549683, -0.6409034132957458, -0.016871731728315353, 0.6311647891998291, 0.035879798233509064};
		float pickupJoints[7] = {-0.08959852159023285, 0.9294207692146301, -1.8222562074661255, -1.0717039108276367, -0.6008989810943604, 0.899803876876831, -0.5889070630073547};
		float prepareJoints[7] = {0.34165704250335693, 1.6997754573822021, -1.5356805324554443, -1.9555771350860596, -2.319523572921753, -0.0016413599951192737, 0.33485254645347595};
		*/
		
		float initJoints[7] = {-1.8570010662078857, 1.223028540611267, 1.8580671548843384, -1.699626088142395, -1.1439096927642822, -0.0016839634627103806, -0.0720016211271286};
		float pickupJoints[7] = {-0.9376608729362488, 1.210081696510315, 2.0494399070739746, -1.2310117483139038, -2.8832123279571533, -0.8106216788291931, -0.018600357696413994};
		float prepareJoints[7] = {-1.434193730354309, 1.2308440208435059, 1.542385458946228, -1.9307560920715332, -2.5853400230407715, -0.06925068795681, -1.7760529518127441};
		float leaveJoints[7] = {-1.5658361911773682, 1.4227105379104614, 1.4938398599624634, -1.8213244676589966, 1.4891080856323242, 0.12021292001008987, 1.5366511344909668};
		
		raHand->connectHand();
		
		raQueue->stopCurrentMode();
		raQueue->switchMode(10);
		
		raQueue->setStiffness(pickStiffnessxyz, pickStiffnessabc, pickDamping, pickMaxDelta, pickMaxForce, pickMaxAxisTorque);
		
//		laQueue->stopCurrentMode();
//		laQueue->switchMode(10);

		raHand->closeHand(0.0, handVelocity);
		
//		TrajectoryDMPLearner catchLearner(catchTmpmys, tmpsigmas, az, bz, catchFile, columns - 1);
//		t_learned_dmp catchDmps = catchLearner.fitTrajectories();
		
//		DMPExecutor catchDmpexec(catchDmps);
//		t_executor_res catchDmpResult;

		
		float leftInitPos[7] = {-0.9235047698020935, 1.0998013019561768, -0.5995920896530151, -0.827528715133667, -1.4004626274108887, -0.36493226885795593, 0.4615746736526489};
	//	0.42438337206840515, 0.8369218111038208, 2.645928144454956, 1.228681206703186, -1.2770203351974487, -1.1407939195632935, -0.1754106879234314
	//	float leftInitPos[7] = {-4.412943363189697, 2.754855155944824, -1.0390863418579102, 2.5718648433685303, 0.4171838164329529, -0.5168850421905518, -0.29127413034439087};
		
		cout << "moving to right arm to start position" << endl;
		raQueue->moveJoints(initJoints);
		
		// not working right now (robot calibration of arm is bad)
//		cout << "moving left arm to start position" << endl;
//		laQueue->moveJoints(leftInitPos);
		
		cout << "moving to pickup" << endl;
		raQueue->moveJoints(pickupJoints);
		
 		raHand->closeHand(1.0, handVelocity);
		
		cout << "moving back to start position" << endl;
		raQueue->moveJoints(initJoints);
		
//		raQueue->setAdditionalLoad(0.15, 135);

		raQueue->moveJoints(prepareJoints);
		
		raThr = raQueue->startQueueThread();
		
		raQueue->switchMode(30);
		
        executeDemo(raQueue, screwFile, doSimulation, az, bz, 1);
		raHand->closeHand(0.0, handVelocity);
		
		raQueue->switchMode(30);
		raQueue->moveJoints(initJoints);
		
		
		raQueue->stopCurrentMode();
		raHand->disconnectHand();
		
		
	} else if(mode == 1) {
 
		ControlQueue* laQueue = NULL;
		thread* raThr = NULL;
 
		// execute guided measurement
		laQueue = new OrocosControlQueue(argc, args, kukaStepWaitTime,
						prefix + "/" + hardware + "/" + "joint_control" + "/" + moveTopic,
						prefix + "/" + hardware + "/" + "joint_control" + "/" + jntPosTopic,
						prefix + "/" + hardware + "/" + "settings" + "/" + switchTopic,
						prefix + "/" + hardware + "/" + "cartesian_control" + "/" + carPosTopic,
						prefix + "/" + hardware + "/" + "cartesian_control" + "/" + setCartImpTopic,
						prefix + "/" + hardware + "/" + "joint_control" + "/" + setJntImpTopic,
					//	prefix + "/" + hardware + "/" + "joint_control" + "/" + jntPtpTopic,
						"joint_control/ptp",
						prefix + "/" + hardware + "/" + "settings" + "/" + getCmdStateTopic,
					//	prefix + "/" + hardware + "/" + "sensoring" + "/" + ptpReachedTopic,
						"joint_control/right_arm/PtpReached",
						prefix + "/" + hardware + "/" + "settings" + "/" + setAddLoadTopic,
				   *node
			      );
 
//		prefix = "Right";
//		laQueue = new OrocosControlQueue(argc, args, kukaStepWaitTime, prefix + string("MoveJoints"), prefix + string("PosJoints"), prefix + string("SwitchMode"),
//						prefix + string("PosCartesian"), prefix + string("SetCartImpedance"), prefix + string("SetJointImpedance"),
//						string("/joint_control/") + prefix + ("_arm/JointPtp"), prefix + string("GetCommandState"), string("/joint_control/") + prefix + ("_arm/PtpReached"), prefix + string("SetAdditionalLoad"), node);
 
		laQueue->stopCurrentMode();
		raThr = laQueue->startQueueThread();
		laQueue->setStiffness(0.2, 0.01, 0.2, 15000, 150, pickMaxAxisTorque);
 
		laQueue->switchMode(30);
 
		ofstream oFile;
		oFile.open(outFile);
 
		double time = 0.0;
		double lastTime = -1.0;
		float* joints;
 
		std::thread* inputThr = NULL;
		inputThr = new std::thread(consoleInputter);
 
		while(consoleInput == 0) {
 
			mes_result mesRes = laQueue->getCurrentJoints();
 
			time = mesRes.time;
			joints = mesRes.joints;
			//cout << joints << " " << lastTime << " " << time << endl;
			usleep(0.5 * 1e4);
			if(joints != NULL && lastTime != time) {
				oFile << time;
				for(int i = 0; i < columns - 1; ++i) { oFile << "\t" << joints[i]; }
				oFile << endl;
				lastTime = time;
 
			}
 
 
		}
 
		laQueue->switchMode(10);
		laQueue->stopCurrentMode();
		
	} else if(mode == 2) {
		
		ControlQueue* laQueue = NULL;
		thread* raThr = NULL;
		
		// execute guided measurement
		laQueue = new OrocosControlQueue(argc, args, kukaStepWaitTime,
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
		
//		prefix = "Right";
//		laQueue = new OrocosControlQueue(argc, args, kukaStepWaitTime, prefix + string("MoveJoints"), prefix + string("PosJoints"), prefix + string("SwitchMode"),
//						prefix + string("PosCartesian"), prefix + string("SetCartImpedance"), prefix + string("SetJointImpedance"),
//						string("/joint_control/") + prefix + ("_arm/JointPtp"), prefix + string("GetCommandState"), string("/joint_control/") + prefix + ("_arm/PtpReached"), prefix + string("SetAdditionalLoad"), node);
		
		laQueue->stopCurrentMode();
		raThr = laQueue->startQueueThread();
		laQueue->setStiffness(0.2, 0.01, 0.2, 15000, 150, pickMaxAxisTorque);
		
		laQueue->switchMode(30);
		
		ofstream oFile;
		oFile.open(outFile);
		
		double time = 0.0;
		double lastTime = -1.0;
		float* joints;
		
		std::thread* inputThr = NULL;
		inputThr = new std::thread(consoleInputter);
		
		while(consoleInput == 0) {
			
			mes_result mesRes = laQueue->getCurrentJoints();
			
			time = mesRes.time;
			joints = mesRes.joints;
			//cout << joints << " " << lastTime << " " << time << endl;
			usleep(0.5 * 1e4);
			if(joints != NULL && lastTime != time) {
				oFile << time;
				for(int i = 0; i < columns - 1; ++i) { oFile << "\t" << joints[i]; }
				oFile << endl;
				lastTime = time;
				
			}
			
			
		}
		
		laQueue->switchMode(10);
		laQueue->stopCurrentMode();
		if(!doSimulation) {
			
			thread* raThr = NULL;
			
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
			
			raQueue->stopCurrentMode();
			raThr = raQueue->startQueueThread();
			raQueue->setStiffness(pickStiffnessxyz, pickStiffnessabc, pickDamping, pickMaxDelta, pickMaxForce, pickMaxAxisTorque);
			
			raQueue->switchMode(30);
		
		}
		
        if(!doSimulation) executeDemo(raQueue, inFile, doSimulation, az, bz, 1);
        else executeDemo(NULL, inFile, doSimulation, az, bz, 1);
		
	} else if(mode == 3) {
		
		testDictionaryGen();
	
	} else if(mode == 4) {
		
		ControlQueue* raQueue = NULL;
		if(!doSimulation) {
			
			thread* raThr = NULL;
			
			// execute guided measurement
			raQueue = new OrocosControlQueue(argc, args, kukaStepWaitTime,
						prefix + "/" + hardware + "/" + "joint_control" + "/" + moveTopic,
						prefix + "/" + hardware + "/" + "joint_control" + "/" + jntPosTopic,
						prefix + "/" + hardware + "/" + "settings" + "/" + switchTopic,
						prefix + "/" + hardware + "/" + "cartesian_control" + "/" + carPosTopic,
						prefix + "/" + hardware + "/" + "cartesian_control" + "/" + setCartImpTopic,
						prefix + "/" + hardware + "/" + "joint_control" + "/" + setJntImpTopic,
					//	prefix + "/" + hardware + "/" + "joint_control" + "/" + jntPtpTopic,
						"joint_control/ptp",
						prefix + "/" + hardware + "/" + "settings" + "/" + getCmdStateTopic,
					//	prefix + "/" + hardware + "/" + "sensoring" + "/" + ptpReachedTopic,
						"joint_control/right_arm/PtpReached",
						prefix + "/" + hardware + "/" + "settings" + "/" + setAddLoadTopic,
				   *node
			      );
			
			raQueue->stopCurrentMode();
			raThr = raQueue->startQueueThread();
			raQueue->setStiffness(5.0, 5.0, 5.0, 15000, 150, pickMaxAxisTorque);
			
			raQueue->switchMode(30);
		
		}
		
		t_executor_res dmpResult;
        if(!doSimulation) dmpResult = executeDemo(raQueue, inFile, doSimulation, az, bz, 1);
        else dmpResult = executeDemo(NULL, inFile, doSimulation, az, bz, 1);
		
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
		
	} else if(mode == 5) {
		
		ControlQueue* laQueue = NULL;
		thread* raThr = NULL;
		
		// execute guided measurement
		laQueue = new OrocosControlQueue(argc, args, kukaStepWaitTime,
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
		
//		prefix = "Right";
//		laQueue = new OrocosControlQueue(argc, args, kukaStepWaitTime, prefix + string("MoveJoints"), prefix + string("PosJoints"), prefix + string("SwitchMode"),
//						prefix + string("PosCartesian"), prefix + string("SetCartImpedance"), prefix + string("SetJointImpedance"),
//						string("/joint_control/") + prefix + ("_arm/JointPtp"), prefix + string("GetCommandState"), string("/joint_control/") + prefix + ("_arm/PtpReached"), prefix + string("SetAdditionalLoad"), node);
		
		laQueue->stopCurrentMode();
		raThr = laQueue->startQueueThread();
		laQueue->setStiffness(0.2, 0.01, 0.2, 15000, 150, pickMaxAxisTorque);
		
		laQueue->switchMode(30);
		
		ofstream oFile;
		oFile.open(outFile);
		
		double time = 0.0;
		double lastTime = -1.0;
		float* joints;
		
		char input = ' ';
		
		while(input != 'q') {
			
			input = getchar();
			cout << joints << " " << lastTime << " " << time << endl;
			
			mes_result mesRes = laQueue->getCurrentJoints();
			
			time = mesRes.time;
			joints = mesRes.joints;

			if(joints != NULL && lastTime != time) {
				oFile << time;
				for(int i = 0; i < columns - 1; ++i) { oFile << "\t" << joints[i]; }
				oFile << endl;
				lastTime = time;
				
			}
			
			
		}
		
		laQueue->switchMode(10);
		laQueue->stopCurrentMode();
		
	} else if(mode == 6) {
		
	//	testMetric();
		testPoWER();
		
	
		
	} else if(mode == 7) {
		
        testIROS();
    //	testIROSGrasping();
		
	}
	
	getch();

	return 0;

}

// TODO: something has been changed such that it is not working anymore (maybe data files?)
void testIROSGrasping() {
	
	ControlQueue* raQueue = NULL;
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
	double relativeDistanceThresh = 0.4;
	
	cout << "tau: " << tau << endl;
	cout << "ax: " << ax << endl;

	vec newQueryPoint(2);
	newQueryPoint(0) = 8;
	newQueryPoint(1) = 8;
	
	// result for (8, 8)
	vector<double> vectorNewQueryPoint = {-0.488059, 1.37839, -2.0144, -1.87255, -0.244649, -0.120394, 1.23313};
	
	GraspingRewardComputer reward(vectorNewQueryPoint);
//	t_executor_res opt = reward.getOptimalTraj(5.0);
	
	cout << "execute ground truth for (8, 8)" << endl;
    t_executor_res opt = executeDemo(NULL, "/home/shangl/iis_robot_sw/iis_catkin_ws/src/kukadu/src/kukadu_core/movements/iros2014/traj_8-8.txt", doSimulation, az, bz, 0);
	
	// speedup testing process by inserting already learned metric result
	mat m(2,2);
	m(0, 0) = 1.0;
	m(1, 0) = -1.3176;
	m(0, 1) = -1.3176;
	m(1, 1) = 3.2793;
//	1.0000  -1.3176
//  -1.3176   3.2793
	
//	dmpGen = new DictionaryGeneralizer(newQueryPoint, raQueue, inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, trajMetricWeights, relativeDistanceThresh, as);
	dmpGen = new DictionaryGeneralizer(newQueryPoint, raQueue, inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, as, m, relativeDistanceThresh);
	
	std::vector<Trajectory*> initTraj;
	initTraj.push_back(dmpGen->getTrajectory());
	
	cout << newQueryPoint << endl;
	cout << "(mainScrewOrocos) first metric: " << ((LinCombDmp*) dmpGen->getTrajectory())->getMetric().getM() << endl;
	
	LinCombDmp* lastRollout = NULL;
	
	PoWER pow(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, &reward, NULL, ac, dmpStepSize, tolAbsErr, tolRelErr);
	
	int plotTimes = 5;
	g1 = new Gnuplot("PoWER demo");
	int i = 0;
	
	vec initT;
	vector<vec> initY;
	
	vector<double> lastRewards;
//	while( lastRewards.size() < 3 || (lastRewards.at(0) != lastRewards.at(1) || lastRewards.at(1) != lastRewards.at(2) ) ) {
	while( i < 8 ) {

		pow.performRollout(1, 0);
		lastRollout = dynamic_cast<LinCombDmp*>(pow.getLastUpdate());
		lastRewards.push_back(pow.getLastUpdateReward());
		if(lastRewards.size() > 3)
			lastRewards.erase(lastRewards.begin());
		
		cout << "(mainScrewOrocos) last update metric: " << lastRollout->getMetric().getM() << endl;
		cout << lastRollout->getMetric().getM() << endl << endl;
		
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
		
		
		lastRollout = ((LinCombDmp*) dmpGen->getTrajectory());
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
	
	ControlQueue* raQueue = NULL;
	QuadraticKernel* kern = new QuadraticKernel();
	
	std::vector<double> irosmys = {0, 1, 2, 3, 4, 5};
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
	
	GaussianObstacleRewardComputer reward(newQueryPoint(0), 2.0, newQueryPoint(1));
	t_executor_res opt = reward.getOptimalTraj(5.0);
	
	if(!doSimulation) {

		thread* raThr = NULL;
		
		raQueue->stopCurrentMode();
		raThr = raQueue->startQueueThread();
		raQueue->setStiffness(pickStiffnessxyz, pickStiffnessabc, pickDamping, pickMaxDelta, pickMaxForce, pickMaxAxisTorque);
		
		raQueue->switchMode(30);
	
	}
	
	dmpGen = new DictionaryGeneralizer(newQueryPoint, raQueue, inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, trajMetricWeights, relativeDistanceThresh, as);
	
	/*
	switchThr = new std::thread(switchQueryPoint);
	t_executor_res genRes = dmpGen->simulateTrajectory();
	
	switchThr->join();
	*/
	
	if(!doSimulation)
		raQueue->stopCurrentMode();
	
	/*
	vector<t_executor_res> results;
	for(int j = 0; j < dmpGen->getQueryPointCount(); ++j) {
		
		cout << "simulating trajectory " << j << endl;
		t_executor_res dmpResult;
		DMPExecutor dmpexec(dmpGen->getQueryPointByIndex(j).getDmp());
		dmpResult = dmpexec.simulateTrajectory(0, dmpGen->getQueryPointByIndex(0).getDmp().getTmax(), dmpStepSize, tolAbsErr, tolRelErr);
		results.push_back(dmpResult);
		
	}
	*/
	
	std::vector<Trajectory*> initTraj;
	initTraj.push_back(dmpGen->getTrajectory());
	
	cout << newQueryPoint << endl;
	cout << "(mainScrewOrocos) first metric: " << ((LinCombDmp*) dmpGen->getTrajectory())->getMetric().getM() << endl;
	
	PoWER pow(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, &reward, NULL, ac, dmpStepSize, tolAbsErr, tolRelErr);
	
	int plotTimes = 5;
	g1 = new Gnuplot("PoWER demo");
	int i = 0;
	
	vec initT;
	vector<vec> initY;
	
	LinCombDmp* lastRollout = NULL;
	vector<double> lastRewards;
//	while( lastRewards.size() < 3 || (lastRewards.at(0) != lastRewards.at(1) || lastRewards.at(1) != lastRewards.at(2) ) ) {
	while( i < 8 ) {

		pow.performRollout(1, 0);
		lastRollout = dynamic_cast<LinCombDmp*>(pow.getLastUpdate());
		lastRewards.push_back(pow.getLastUpdateReward());
		if(lastRewards.size() > 3)
			lastRewards.erase(lastRewards.begin());
		
	//	cout << "(mainScrewOrocos) last update metric: " << lastRollout->getMetric().getM() << endl;
		cout << lastRollout->getMetric().getM() << endl << endl;
		
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
		t_executor_res opt2 = reward2.getOptimalTraj(5.0);
		
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
		t_executor_res opt3 = reward3.getOptimalTraj(5.0);
		
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

void testDictionaryGen() {
	
	ControlQueue* raQueue = NULL;
	QuadraticKernel* kern = new QuadraticKernel();
	
	vec trajMetricWeights(7);
	trajMetricWeights.fill(1.0);
	
	tau = 15;
	double ax = -log((float)0.1) / tau / tau;
	
	cout << "tau: " << tau << endl;
	cout << "ax: " << ax << endl;
	
	vec newQueryPoint(1);
	//newQueryPoint(0) = 6.5;
	newQueryPoint(0) = 6.0;
	
	double kernParam = 1.0;
	
	if(!doSimulation) {
		
		thread* raThr = NULL;
		
		raQueue->stopCurrentMode();
		raThr = raQueue->startQueueThread();
		raQueue->setStiffness(pickStiffnessxyz, pickStiffnessabc, pickDamping, pickMaxDelta, pickMaxForce, pickMaxAxisTorque);
		
		raQueue->switchMode(30);
	
	}
	
	dmpGen = new DictionaryGeneralizer(newQueryPoint, raQueue, inDir, columns - 1, genTmpmys, tmpsigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, trajMetricWeights, 0.2, as);
	
	switchThr = new std::thread(switchQueryPoint);
	t_executor_res genRes = dmpGen->simulateTrajectory();
	
	switchThr->join();
	
	if(!doSimulation)
		raQueue->stopCurrentMode();
	
	/*
	vector<t_executor_res> results;
	for(int j = 0; j < dmpGen->getQueryPointCount(); ++j) {
		
		cout << "simulating trajectory " << j << endl;
		t_executor_res dmpResult;
		DMPExecutor dmpexec(dmpGen->getQueryPointByIndex(j).getDmp());
		dmpResult = dmpexec.simulateTrajectory(0, dmpGen->getQueryPointByIndex(0).getDmp().getTmax(), dmpStepSize, tolAbsErr, tolRelErr);
		results.push_back(dmpResult);
		
	}
	*/
	
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
	
}

void testPoWER() {
	
	double as = 0.5;
	double az = 48.0;
	double bz = (az - 1) / 4;
	double handVelocity = 20.0;
	
	double ax = 0.1;
	double tau = 0.8;

	float pickStiffnessxyz = 1500;
	float pickStiffnessabc = 500;
	float pickDamping = 1.0;
	float pickMaxDelta = 99;
	float pickMaxForce = 150;
	float pickMaxAxisTorque = 2.0;

	int kukaStepWaitTime = 1.8 * 1e4;
	double dmpStepSize = kukaStepWaitTime * 1e-6;

	// constant for phase stopping
	double ac = 10;
	
	double tolAbsErr = 10;
	double tolRelErr = 10;
	
	double explorationSigma = 0.1;
	double sig = 200;
	vector<double> exploreSigmas = {sig, sig, sig, sig, sig, sig, sig, sig, sig};
	int updatesPerRollout = 50;
	int importanceSamplingCount = 3;
	double t_max = 7;
	
	SampleRewardComputer rew(0.1);
	t_executor_res opt = rew.getOptimalTraj(t_max);
	double deltaT = 0.05;
	int tCount = opt.t.n_elem;
	mat sample(tCount, 2);
	sample.fill(0.0);
	
	double currentT = 0.0;
	double k = 0.5;
	double A = 0.2;
	
	vec s = opt.y.at(0);
	for(int i = 0; i < s.n_elem; ++i) {
		sample(i, 0) = opt.t(i);
		float tmp =  0.1 * i;
		sample(i, 1) = s(i) * 1 / exp(tmp);
	}
	
	vector<double> mys = constructDmpMys(sample);	
	vector<double> sigmas{0.3};
	
	vector<DMPBase> baseDef = buildDMPBase(mys, sigmas, ax, tau);
	TrajectoryDMPLearner learner(baseDef, tau, az, bz, ax, sample, 1);
	Dmp initialDmp = learner.fitTrajectories();

	vector<Trajectory*> initDmp;
	initDmp.push_back(&initialDmp);
	
	getchar();
	
	DMPExecutor exec(initialDmp);
	PoWER pow(&exec, initDmp, exploreSigmas, updatesPerRollout, importanceSamplingCount, &rew, NULL, ac, dmpStepSize, tolAbsErr, tolRelErr);
	
	int plotTimes = 5;
	g1 = new Gnuplot("PoWER demo");
	int i = 0;
	while( 1 ) {

		pow.performRollout(1, 0);
		Trajectory* lastRollout = pow.getLastUpdate();
		
		if( (i % 1) == 0 ) {
			
			cout << "(mainScrewOrocos) already done " << i << " iterations" << endl;
		
			int plotNum = 1;
			for(int plotTraj = 0; plotTraj < plotNum; ++plotTraj) {
				
				ostringstream convert;   // stream used for the conversion
				convert << plotTraj;
				
				string title = string("fitted sensor data (joint") + convert.str() + string(")");
				g1->set_style("lines").plot_xy(armadilloToStdVec(pow.getLastUpdateRes().t), armadilloToStdVec(pow.getLastUpdateRes().y[plotTraj]), "dmp y");
				g1->set_style("lines").plot_xy(armadilloToStdVec(opt.t), armadilloToStdVec(opt.y[plotTraj]), "dmp y");
				g1->showonscreen();
				
			}
			
		}
		
		g1->reset_plot();
		
		if( (i % 20) == 19)
			g1->remove_tmpfiles();
		
		++i;
		
	}
	
}

void testTrajectoryMetric() {
/*
	// test for trajectory distance computation
	double dist = 0.0;
	vec degOfFreedomWeights(7);
	degOfFreedomWeights.fill(1.0);
	
	
	cout << "compare 0 and 1" << endl;
	DMPTrajectoryComparator comp(results.at(0), results.at(1), degOfFreedomWeights);
	dist = comp.computeDistance();
	cout << "distance: " << dist << endl;

	// double integrationStep, double tolAbsErr, double tolRelErr
	cout << "compare 1 and 2" << endl;
	comp.setTrajectories(dmpGen->getQueryPointByIndex(1).getDmp(), dmpGen->getQueryPointByIndex(2).getDmp(), dmpStepSize, tolAbsErr, tolRelErr, 0.2);
//		DMPTrajectoryComparator comp(dmpGen->getQueryPointByIndex(1).getDmp(), dmpGen->getQueryPointByIndex(2).getDmp(), degOfFreedomWeights, dmpStepSize, tolAbsErr, tolRelErr, 0.2);
	dist = comp.computeDistance();
	
	cout << "distance: " << dist << endl;
	
	cout << "compare 1 and 3" << endl;
	comp.setTrajectories(dmpGen->getQueryPointByIndex(3).getDmp(), dmpGen->getQueryPointByIndex(1).getDmp(), dmpStepSize, tolAbsErr, tolRelErr, 0.2);
	dist = comp.computeDistance();
	
	cout << "distance: " << dist << endl;
	
	cout << "compare 0 and 2" << endl;
	comp.setTrajectories(dmpGen->getQueryPointByIndex(0).getDmp(), dmpGen->getQueryPointByIndex(2).getDmp(), dmpStepSize, tolAbsErr, tolRelErr, 0.2);
	dist = comp.computeDistance();
	
	cout << "distance: " << dist << endl;
	
	cout << "compare 1 and 3" << endl;
	comp = DMPTrajectoryComparator(dmpGen->getQueryPointByIndex(3).getDmp(), dmpGen->getQueryPointByIndex(1).getDmp(), degOfFreedomWeights, dmpStepSize, tolAbsErr, tolRelErr, 0.2);
	dist = comp.computeDistance();
	
	cout << "distance: " << dist << endl;
*/
}

void testMetric() {
	
	cout << "(mainScrewOrocos) testing section" << endl;
		
	vector<vec> x1s, x2s;
	vector<double> distances;
	
	int grid = 10;
	
	for(int i = 0; i < grid; ++i) {
		
		for(int j = 0; j < grid; ++j) {
			
			vec x1(2);
			x1(0) = i;
			x1(1) = j;
			
			for(int k = 0; k < grid; ++k) {
				
				for(int l = 0; l < grid; ++l) {
					
					vec x2(2);
					x2(0) = k;
					x2(1) = l;
					
					x1s.push_back(x1);
					x2s.push_back(x2);
					
					vec distTmp = (x1 - x2).t() * (x1 - x2);
					double dist = distTmp(0);
					
				//	double dist = rand() % 10;
					
					distances.push_back(dist);
					
				}
			}
		}
	}


//	InfTheoMetricLearner learner(x1s, x2s, distances, 3.0, 6.0, 1.0, 0.2, 40);
	TogersonMetricLearner learner(x1s, x2s, distances);
	Mahalanobis metric = learner.learnMetric();
	cout << "metric: " << metric.getM() << endl;
	
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
