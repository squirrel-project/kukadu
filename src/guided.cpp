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
#include <boost/program_options.hpp>

#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include "../include/kukadu.h"
#include "../src/utils/gnuplot-cpp/gnuplot_i.hpp"

#define DOSIMULATION 1

using namespace std;
using namespace arma;
namespace po = boost::program_options;

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
std::string hand = "left";


string hardware = left_hardware;

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

string handPort = "/dev/ttyUSB1";
// char* screwFile = "/home/shangl/catkin_ws/src/kukadu/src/kukadu_core/movements/orocos_demo/screw3.txt";
// char* screwFile = "/home/shangl/leftscrew.txt";
string screwFile = "/home/shangl/newest.txt";
// char* screwFile = "/home/shangl/blub.txt";

string storeFolder = "/home/c7031109/tmp/pushTransRightToLeft";


// with current implementation tStart has to be 0.0
double tStart = 0.0;
double tEnd = 7.5;

int main(int argc, char** args) {

    int importanceSamplingCount;
    double tau, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ac, as, alpham;
    string inDir, cfFile, dataFolder, trajFile;
    vector<double> rlExploreSigmas;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("metric.database", po::value<string>(), "database folder")
            ("metric.goldStandard", po::value<string>(), "gold standard file")
            ("metric.alpham", po::value<double>(), "alpham")
            ("metric.exploreSigmas", po::value<string>(), "reinforcement learning exploration sigmas")
            ("metric.importanceSamplingCount", po::value<int>(), "size of importance sampling vector")
            ("metric.as", po::value<double>(), "as")
            ("dmp.tau", po::value<double>(), "tau")
            ("dmp.az", po::value<double>(), "az")
            ("dmp.bz", po::value<double>(), "bz")
            ("dmp.dmpStepSize", po::value<double>(), "dmp time step size")
            ("dmp.tolAbsErr", po::value<double>(), "tolerated absolute error")
            ("dmp.tolRelErr", po::value<double>(), "tolerated relative error")
            ("dmp.ac", po::value<double>(), "ac")
            ("mes.folder", po::value<string>(), "measurment data folder")
            ("pick.trajectory", po::value<string>(), "data for measured trajectory")
    ;

    ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/book_pick.prop"), std::ifstream::in);
    po::variables_map vm;
    po::store(po::parse_config_file(parseFile, desc), vm);
    po::notify(vm);

    if (vm.count("dmp.tau")) tau = vm["dmp.tau"].as<double>();
    else return 1;
    if (vm.count("dmp.az")) az = vm["dmp.az"].as<double>();
    else return 1;
    if (vm.count("dmp.bz")) bz = vm["dmp.bz"].as<double>();
    else return 1;
    if (vm.count("dmp.dmpStepSize")) dmpStepSize = vm["dmp.dmpStepSize"].as<double>();
    else return 1;
    if (vm.count("dmp.tolAbsErr")) tolAbsErr = vm["dmp.tolAbsErr"].as<double>();
    else return 1;
    if (vm.count("dmp.tolRelErr")) tolRelErr = vm["dmp.tolRelErr"].as<double>();
    else return 1;
    if (vm.count("dmp.ac")) ac = vm["dmp.ac"].as<double>();
    else return 1;

    if (vm.count("metric.as")) as = vm["metric.as"].as<double>();
    else return 1;
    if (vm.count("metric.database")) inDir = resolvePath(vm["metric.database"].as<string>());
    else return 1;
    if (vm.count("metric.goldStandard")) cfFile = resolvePath(vm["metric.goldStandard"].as<string>());
    else return 1;
    if (vm.count("metric.alpham")) alpham = vm["metric.alpham"].as<double>();
    else return 1;
    if (vm.count("metric.exploreSigmas")) {
        string tokens = vm["metric.exploreSigmas"].as<string>();
        stringstream parseStream(tokens);
        char bracket;
        double dToken;
        parseStream >> bracket;
        while(bracket != '}') {
            parseStream >> dToken >> bracket;
            rlExploreSigmas.push_back(dToken);
        }
    } else return 1;
    if (vm.count("metric.importanceSamplingCount")) importanceSamplingCount = vm["metric.importanceSamplingCount"].as<int>();
    else return 1;

    if (vm.count("mes.folder")) dataFolder = resolvePath(vm["mes.folder"].as<string>());
    else return 1;

    if (vm.count("pick.trajectory")) trajFile = resolvePath(vm["pick.trajectory"].as<string>());
    else return 1;

    cout << "all properties loaded" << endl;
    int kukaStepWaitTime = dmpStepSize * 1e6;

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    shared_ptr<ControlQueue> leftQueue = shared_ptr<ControlQueue>(new KukieControlQueue(kukaStepWaitTime, prefix, hardware, *node));
    vector<shared_ptr<ControlQueue>> queueVectors;
    queueVectors.push_back(leftQueue);

    leftQueue->stopCurrentMode();
    std::shared_ptr<std::thread> raThr = leftQueue->startQueueThread();

    leftQueue->setStiffness(0.2, 0.01, 0.2, 15000, 150, 1500);

    leftQueue->switchMode(30);

    SensorStorage scaredOfSenka(queueVectors, std::vector<std::shared_ptr<GenericHand>>(), 1000);
    scaredOfSenka.setExportMode(STORE_TIME | STORE_RBT_CART_POS | STORE_RBT_JNT_POS);
    scaredOfSenka.startDataStorage(storeFolder);
    ros::Rate r(1);
    for(int i = 0; i < 20; ++i) {
        r.sleep();
        cout << i << endl;
    }
    scaredOfSenka.stopDataStorage();

    leftQueue->stopCurrentMode();

}


char consoleInputter() {
    cin >> consoleInput;
    return consoleInput;
}
