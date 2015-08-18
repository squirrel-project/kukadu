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


int main(int argc, char ** args) {
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
    cout << "hand interface created" << endl;

    //moving hand to staring position

    vector<double> newPos = {0, -1.57, 0, -1.57, 0, -1.57, 0};
    handQ->publishSdhJoints(newPos);


    //moving arm

    leftQueue->stopCurrentMode();
    shared_ptr<thread> lqThread = leftQueue->startQueueThread();
    leftQueue->switchMode(10);


    //string loadDataPath = "/home/c7031098/testing/test/kuka_lwr_real_left_arm_01";
    //string loadDataPath = "/home/c7031109/tmp/pushing_data/kuka_lwr_real_left_arm_0";
    string loadDataPath ="/home/c7031098/testing/testDataJoint/data02/kuka_lwr_simulation_left_arm_0";
    shared_ptr<SensorData> data = SensorStorage::readStorage(leftQueue, loadDataPath);
    data->removeDuplicateTimes();


    //loading data for learner

    arma::vec times = data->getTimes();
    arma::mat jointPos = data->getJointPos();
    arma::mat cartPos = data->getCartPos();
    cout <<" (testing) data loaded" <<endl;

   // cout << jointPos.row(0) << endl;
    leftQueue->moveJoints(jointPos.row(0).t());

    leftQueue->stopCurrentMode();
    leftQueue->setStiffness(2000, 150, 0.7, 7.0,70.0, 2.0);
    leftQueue->switchMode(20);

    // JointDMPLearner learner(az, bz, join_rows(times, cartPos));

    CartesianDMPLearner learner(az, bz, join_rows(times, cartPos));
    std::shared_ptr<Dmp> leftDmp = learner.fitTrajectories();
    DMPExecutor leftExecutor(leftDmp, leftQueue);
    shared_ptr<thread> storeThread;


    if(environment == "simulation"){

        //creatin simulation interface

        std::shared_ptr<SimInterface> simI= std::shared_ptr<SimInterface>(new SimInterface (argc, args, kukaStepWaitTime, *node));
        cout<<"simulator interface created"<<endl;

        float newP[3] = {0.5, 0.7, 0.01};
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

        //collecting data

        SensorStorage storeData(queueVectors, std::vector<std::shared_ptr<GenericHand>>(), simI, objectId, 1000);
        storeData.setExportMode(STORE_TIME | STORE_RBT_CART_POS | STORE_RBT_JNT_POS);
        // string loadDataPath = "/home/c7031098/testing/SimData/push0709/execution1_box/";
        //string saveDataPath = "/home/c7031098/testing/testDataJoint/data02/";
        //storeThread = storeData.startDataStorage(saveDataPath);
        leftExecutor.executeTrajectory(ac, 0, leftDmp->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);
        leftQueue->stopCurrentMode();
        storeData.stopDataStorage();

    }

    if(environment == "real"){

        //vision interface

        shared_ptr<VisionInterface> vis = shared_ptr<VisionInterface>(new VisionInterface(argc, args, kukaStepWaitTime, *node));
        cout<<"vision interface created"<<endl;

        vis->setArTagTracker();
        cout<<"ArTagTracke started"<<endl;


        //collecting data

        SensorStorage storeData(queueVectors, std::vector<std::shared_ptr<GenericHand>>(), vis, 1000);
        storeData.setExportMode(STORE_TIME | STORE_RBT_CART_POS | STORE_RBT_JNT_POS | STORE_RBT_JNT_FTRQ);
        // string filePath = "/home/c7031098/testing/Push0709/execution2_box/";
        string filePath = "/home/c7031109/tmp/blub";
        storeThread = storeData.startDataStorage(filePath);

        cout <<" (collectData) executing starting" <<endl;

        leftExecutor.executeTrajectory(ac, 0, leftDmp->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);
        leftQueue->stopCurrentMode();
        cout<<" (collectData) execution done "<<endl;
        storeData.stopDataStorage();

    }

    storeThread->join();

    cout<<"(collectData) moving to start position "<<endl;

    leftQueue->stopCurrentMode();
//    leftQueue->switchMode(10);
//    leftQueue->moveJoints(jointPos.row(0).t());
//    leftQueue->stopCurrentMode();

    getchar();

    return 0;


}
