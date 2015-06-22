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

    cout << "0" << args[0] << endl;
    cout << "1" << args[1] << endl;
    cout << "2" << args[2] << endl;

    ros::init(argc, args, "kukadu"); node = new ros::NodeHandle(); usleep(1e6);

    if(!strcmp(args[1], "right") || !strcmp(args[1], "left"))
        hardware = string(args[1]) + "_arm";
    else
        throw "(mainScrewOrocos) arm not defined";

    outFile = args[2];
    cout << "outfile: " << outFile << endl;

    std::shared_ptr<KukieControlQueue> laQueue = std::shared_ptr<KukieControlQueue>(nullptr);
    std::shared_ptr<std::thread> raThr = std::shared_ptr<std::thread>(nullptr);

    // execute guided measurement
    laQueue = std::shared_ptr<KukieControlQueue>(new KukieControlQueue(kukaStepWaitTime, prefix, hardware, *node));

    laQueue->stopCurrentMode();
    raThr = laQueue->startQueueThread();
    laQueue->setStiffness(0.2, 0.01, 0.2, 15000, 150, pickMaxAxisTorque);

    laQueue->switchMode(30);

    ofstream oFile;
    oFile.open(outFile);

    double time = 0.0;
    double lastTime = -1.0;
    arma::vec joints;

    std::thread* inputThr = NULL;
    inputThr = new std::thread(consoleInputter);

    while(consoleInput == 0) {

        mes_result mesRes = laQueue->getCurrentJoints();

        time = mesRes.time;
        joints = mesRes.joints;

        usleep(0.5 * 1e4);
        if(joints.n_elem > 1 && lastTime != time) {

            oFile << time;
            for(int i = 0; i < columns - 1; ++i) { oFile << "\t" << joints[i]; }
            oFile << endl;
            lastTime = time;

        }


    }

    laQueue->stopCurrentMode();
    laQueue->switchMode(10);
    laQueue->stopCurrentMode();

    getchar();

    return 0;

}


char consoleInputter() {
    cin >> consoleInput;
    return consoleInput;
}
