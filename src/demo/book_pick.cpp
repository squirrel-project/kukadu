#include "../utils/easyloggingpp/src/easylogging++.h"
_INITIALIZE_EASYLOGGINGPP

#include <iostream>
#include <string>
#include <armadillo>
#include <thread>
#include <vector>
#include <boost/program_options.hpp>

#include "../../include/kukadu.h"


#define ROBOT_TYPE "simulation"
#define ROBOT_SIDE "left"

using namespace std;
using namespace arma;
namespace po = boost::program_options;

int main(int argc, char** args) {

    int importanceSamplingCount;
    double tau, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ac, as, alpham;
    string inDir, cfFile, dataFolder;
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
    ;

    ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/book.prop"), std::ifstream::in);
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

    cout << "all properties loaded" << endl;

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);

    int kukaStepWaitTime = dmpStepSize * 1e6;
    shared_ptr<OrocosControlQueue> leftQueue = shared_ptr<OrocosControlQueue>(new OrocosControlQueue(argc, args, kukaStepWaitTime, ROBOT_TYPE, ROBOT_SIDE + string("_arm"), *node));
    shared_ptr<thread> lqThread = leftQueue->startQueueThread();

    shared_ptr<OrocosControlQueue> rightQueue = shared_ptr<OrocosControlQueue>(new OrocosControlQueue(argc, args, kukaStepWaitTime, ROBOT_TYPE, "right_arm", *node));
    shared_ptr<thread> rqThread = rightQueue->startQueueThread();

    usleep(1e6);

    /*

    rightQueue->switchMode(OrocosControlQueue::KUKA_JNT_IMP_MODE);
    rightQueue->moveJoints(stdToArmadilloVec({-1.3607046604156494, 1.1623351573944092, -2.9648025035858154, 1.6621724367141724, -2.3747220039367676, -1.8863340616226196, -0.09742309153079987}));

    // measured joints were -0.2252   1.3174  -2.1671   0.4912   0.8510  -1.5699   1.0577
    mes_result currentJoints = leftQueue->getCurrentJoints();
    cout << currentJoints.joints.t() << endl;

    */

    leftQueue->setStiffness(0.2, 0.01, 0.2, 15000, 150, 2.0);
    leftQueue->switchMode(OrocosControlQueue::KUKA_JNT_IMP_MODE);
    leftQueue->moveJoints(stdToArmadilloVec({-0.4801180064678192, 1.4285844564437866, 1.2313963174819946, -0.9415789246559143, 0.6571714282035828, 1.3351185321807861, 0.820398211479187}));

    shared_ptr<RosSchunk> leftHand = shared_ptr<RosSchunk>(new RosSchunk(*node, ROBOT_TYPE, ROBOT_SIDE));
    vector<double> handJoints = {0, -0.38705404571511043, 0.7258474179992682, 0.010410616072391092, -1.2259735578027993, -0.4303327436948519, 0.8185967300722126};
    leftHand->publishSdhJoints(handJoints);

    cout << "(main) press key to continue" << endl;
    getchar();

    vector<shared_ptr<ControlQueue>> queues = {leftQueue};
    vector<shared_ptr<GenericHand>> hands = {leftHand};
    SensorStorage store(queues, hands, 100);
    shared_ptr<thread> storageThread = store.startDataStorage(dataFolder);

    if(storageThread)
        storageThread->join();

    leftQueue->setFinish();
    rightQueue->setFinish();

    lqThread->join();
    rqThread->join();

}
