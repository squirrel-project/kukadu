#include <ctime>
#include <cmath>
#include <cstdio>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <ros/ros.h>
#include <RedundantKin.h>
#include <std_msgs/Int32.h>
#include <boost/program_options.hpp>

#include "../../include/kukadu/kukadu.h"

#define DOSIMULATION 1

using namespace std;
using namespace arma;
namespace po = boost::program_options;

int main(int argc, char** args) {

    double tau, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ac;

    string storeDir = resolvePath("/tmp/kukadu_demo_guided");
    string prefix = "simulation";
    string right_hardware = "right_arm";

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("dmp.tau", po::value<double>(), "tau")
            ("dmp.az", po::value<double>(), "az")
            ("dmp.bz", po::value<double>(), "bz")
            ("dmp.dmpStepSize", po::value<double>(), "dmp time step size")
            ("dmp.tolAbsErr", po::value<double>(), "tolerated absolute error")
            ("dmp.tolRelErr", po::value<double>(), "tolerated relative error")
            ("dmp.ac", po::value<double>(), "ac")
    ;

    ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/guided.prop").c_str(), std::ifstream::in);
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

    cout << "all properties loaded" << endl;

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    KUKADU_SHARED_PTR<ControlQueue> leftQueue = KUKADU_SHARED_PTR<ControlQueue>(new KukieControlQueue(dmpStepSize, prefix, "left_arm", *node));
    KUKADU_SHARED_PTR<ControlQueue> simLeftQueue = KUKADU_SHARED_PTR<ControlQueue>(new KukieControlQueue(dmpStepSize, "simulation", "left_arm", *node));
    vector<KUKADU_SHARED_PTR<ControlQueue> > queueVectors;
    queueVectors.push_back(leftQueue);

    deleteDirectory(storeDir);

    cout << "press enter to measure trajectory" << endl;
    getchar();

    leftQueue->stopCurrentMode();
    KUKADU_SHARED_PTR<kukadu_thread> laThr = leftQueue->startQueueThread();

    if(!prefix.compare("simulation")) {
        leftQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    } else {
        leftQueue->setStiffness(0.2, 0.01, 0.2, 15000, 150, 1500);
        leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }

    cout << "starting measurement" << endl;
    SensorStorage scaredOfSenka(queueVectors, std::vector<KUKADU_SHARED_PTR<GenericHand> >(), 1000);
    scaredOfSenka.setExportMode(STORE_TIME | STORE_RBT_CART_POS | STORE_RBT_JNT_POS);
    scaredOfSenka.startDataStorage(storeDir);
    cout << "measurment started" << endl;

    if(!prefix.compare("simulation")) {
        cout << "moving arm in simulation" << endl;
        leftQueue->moveJoints(createJointsVector(-0.9692263007164001, 1.113829493522644, 1.1473214626312256, -1.444376826286316, -0.28663957118988037, -0.8957559466362, -0.2651996612548828));
        cout << "movement done" << endl;
    } else {
        ros::Rate r(1);
        for(int i = 0; i < 15; ++i) {
            r.sleep();
            cout << i << endl;
        }
    }

    scaredOfSenka.stopDataStorage();
    leftQueue->stopCurrentMode();

    cout << "press enter to execute in simulation" << endl;
    getchar();

    KUKADU_SHARED_PTR<Dmp> dmpFinalPush;
    KUKADU_SHARED_PTR<SensorData> dataFinalPush;
    KUKADU_SHARED_PTR<JointDMPLearner> learnerFinalPush;

    arma::vec timesFinalPush;

    dataFinalPush = SensorStorage::readStorage(simLeftQueue, storeDir + "/kuka_lwr_simulation_left_arm_0");
    timesFinalPush = dataFinalPush->getTimes();
    learnerFinalPush = KUKADU_SHARED_PTR<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesFinalPush, dataFinalPush->getJointPos())));
    dmpFinalPush = learnerFinalPush->fitTrajectories();

    // simLeftQueue->setStiffness(KukieControlQueue::KUKA_STD_XYZ_STIFF, KukieControlQueue::KUKA_STD_ABC_STIFF, KukieControlQueue::KUKA_STD_CPDAMPING, 15000, 150, 1500);
    simLeftQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    laThr = simLeftQueue->startQueueThread();
    DMPExecutor execFinalPush(dmpFinalPush, simLeftQueue);
    execFinalPush.executeTrajectory(ac, 0, dmpFinalPush->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);

    simLeftQueue->stopCurrentMode();
    simLeftQueue->setFinish();
    laThr->join();

    cout << "press enter to execute on robot" << endl;
    getchar();

    // leftQueue->setStiffness(KukieControlQueue::KUKA_STD_XYZ_STIFF, KukieControlQueue::KUKA_STD_ABC_STIFF, KukieControlQueue::KUKA_STD_CPDAMPING, 15000, 150, 1500);
    leftQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    laThr = leftQueue->startQueueThread();
    DMPExecutor execFinalPush2(dmpFinalPush, leftQueue);
    execFinalPush2.executeTrajectory(ac, 0, dmpFinalPush->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);

    leftQueue->stopCurrentMode();
    leftQueue->setFinish();
    laThr->join();

}
