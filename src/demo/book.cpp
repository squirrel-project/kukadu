#include "../utils/easyloggingpp/src/easylogging++.h"
_INITIALIZE_EASYLOGGINGPP

#include <iostream>
#include <string>
#include <armadillo>
#include <thread>
#include <vector>
#include <boost/program_options.hpp>

#include "../../include/kukadu.h"

using namespace std;
using namespace arma;
namespace po = boost::program_options;

int main(int argc, char** args) {

    int importanceSamplingCount;
    double tau, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ac, as, alpham;
    double handVelocity = 20.0;
    string inDir, cfFile;
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
    ;

    ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/pouring.prop"), std::ifstream::in);
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

    cout << "all loaded" << endl;

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);

    int kukaStepWaitTime = dmpStepSize * 1e6;
    shared_ptr<OrocosControlQueue> leftQueue = shared_ptr<OrocosControlQueue>(new OrocosControlQueue(argc, args, kukaStepWaitTime, "real", "left_arm", *node));
    shared_ptr<thread> lqThread = leftQueue->startQueueThread();

    usleep(1e6);
    mes_result currentJoints = leftQueue->getCurrentJoints();

    /*
    // measured joints were -0.2252   1.3174  -2.1671   0.4912   0.8510  -1.5699   1.0577
    leftQueue->switchMode(10);
    leftQueue->moveJoints(stdToArmadilloVec({-0.2252, 1.3174, -2.1671, 0.4912, 0.8510, -1.5699, 1.0577}));
    cout << currentJoints.joints.t() << endl;
    */

    shared_ptr<RosSchunk> leftHand = shared_ptr<RosSchunk>(new RosSchunk(*node, "real", "left"));

    leftHand->setGrasp(eGID_PARALLEL);
    leftHand->closeHand(0.0, handVelocity);

    vector<double> handJoints = {SDH_IGNORE_JOINT, -1.5, -1.0, 0, -0.2, -1.5, -1};
    leftHand->publishSdhJoints(handJoints);

    /*
    int idx = 0;
    double number = 0.0;
    while(true) {
        cin >> idx;
        cin >> number;
        leftHand->publishSingleJoint(idx, number);
    }
    */

    cout << "(main) press key to continue" << endl;
    getchar();

    vector<shared_ptr<ControlQueue>> queues = {leftQueue};
    vector<shared_ptr<GenericHand>> hands = {leftHand};
    SensorStorage store(queues, hands, 100);
    shared_ptr<thread> storageThread = store.startDataStorage("/home/c7031109/mes_data/nonlinear_bottom_b_2");

    sleep(1);

    handJoints = {SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, 0.4, 1.2, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
    leftHand->publishSdhJoints(handJoints);

    if(storageThread)
        storageThread->join();

    leftQueue->setFinish();
    lqThread->join();

}
