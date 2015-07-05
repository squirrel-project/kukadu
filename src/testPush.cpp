
#include "../include/kukadu.h"

#include <memory>
#include <iostream>
#include <boost/program_options.hpp>

namespace po = boost::program_options;


int importanceSamplingCount;
double tau, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ac, as, alpham;
string inDir, cfFile, dataFolder, trajFile;
vector<double> rlExploreSigmas;

string environment = "simulation";
std::string hand = "left";


int main(int argc, char** args) {
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
    shared_ptr<ControlQueue> leftQueue = shared_ptr<ControlQueue>(new KukieControlQueue(kukaStepWaitTime, environment, "left_arm", *node));
    vector<shared_ptr<ControlQueue>> queueVectors;
    queueVectors.push_back(leftQueue);

    RosSchunk* handQ=new RosSchunk(*node, environment, hand);
    cout<<"hand interface created"<<endl;

    //moving hand to staring position
    vector<double> newPos = {0, -1.57, 0, -1.57, 0, -1.57, 0};
    handQ->publishSdhJoints(newPos);


    //moving arm
    leftQueue->stopCurrentMode();

    shared_ptr<thread> lqThread = leftQueue->startQueueThread();
    leftQueue->switchMode(10);

    shared_ptr<SensorData> data = SensorStorage::readStorage(leftQueue, "/home/c7031098/testing/SimData/pushing_data1/kuka_lwr_simulation_left_arm_0");
    data->removeDuplicateTimes();

    shared_ptr<SensorData> dataO = SensorStorage::readStorage(leftQueue, "/home/c7031098/testing/SimData/pushing_data1/simulation");
    dataO->removeDuplicateTimes();


    //loading data for learner
    arma::vec times = data->getTimes();
    arma::mat jointPos = data->getJointPos();
    arma::mat cartPos = data->getCartPos();
    cout <<"  data loaded" <<endl;

    leftQueue->moveJoints(jointPos.row(0).t());

    //creatin simulation interface
    std::shared_ptr<SimInterface> simI= std::shared_ptr<SimInterface>(new SimInterface (argc, args, kukaStepWaitTime, *node));
    cout<<"simulator interface created"<<endl;

 //   float newP[3] = {0.5, 0.7, 0.01}; //orig
    float newP[3] = {0.6, 0.7, 0.01};
    float newO[4] = {0, 0, 0, 0};
    float dim[3] = {1, 2 , 0.08};

    simI->addPrimShape(1,"sponge", newP, newO, dim, 10.0);
    simI->setObjMaterial("sponge","highFrictionMaterial");
    cout<< "sponge imported "<< endl;

    float newP1[3] = {0.30, 0.7, 0.2};
    float newO1[4] = {0, 0, 0, 0};
    float dim1[3] = {0.2, 0.2, 0.1};

    string objectId = "box";
    simI->addPrimShape(1, objectId, newP1, newO1, dim1, 1);
    cout <<"box imported " << endl;

    leftQueue->stopCurrentMode();
    leftQueue->switchMode(20);

    // JointDMPLearner learner(az, bz, join_rows(times, cartPos));
    CartesianDMPLearner learner(az, bz, join_rows(times, cartPos));
    std::shared_ptr<Dmp> leftDmp = learner.fitTrajectories();
    DMPExecutorPush leftExecutor(leftDmp, leftQueue, environment, simI, objectId);
    leftExecutor.setObjectData(dataO->getTimes(), dataO->getCartPos() );


    SensorStorage storeData(queueVectors, std::vector<std::shared_ptr<GenericHand>>(), simI, objectId, 1000);
    storeData.setExportMode(STORE_TIME | STORE_RBT_CART_POS | STORE_RBT_JNT_POS);
    storeData.startDataStorage("/home/c7031098/testing/SimData/performing_push1/");

    leftExecutor.executeTrajectory(ac, 0, leftDmp->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);
    storeData.stopDataStorage();


    leftQueue->stopCurrentMode();


}
