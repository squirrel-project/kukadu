#include "../include/kukadu.h"

#include <memory>
#include <iostream>
#include <Python.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "learning/projective_simulation/core/psevaluator.h"
#include "learning/projective_simulation/core/projectivesimulator.h"
#include "learning/projective_simulation/application/neverendingcolorreward.h"

namespace po = boost::program_options;
namespace pf = boost::filesystem;

using namespace std;

string environment = "simulation";

string tmpDataFolder = resolvePath("$HOME/tmp/");
string pushDataFolder = resolvePath("$KUKADU_HOME/movements/icra2016/");

int doHapticTest(std::shared_ptr<ControlQueue> leftQueue, shared_ptr<RosSchunk> leftHand);

void goToStartPos(std::shared_ptr<ControlQueue> leftQueue);
void goToBlockingPos(std::shared_ptr<ControlQueue> rightQueue);

void learnRotHor(std::shared_ptr<ControlQueue> leftQueue);
void learnRotVert(std::shared_ptr<ControlQueue> leftQueue);
void learnTransRight(std::shared_ptr<ControlQueue> leftQueue);
void learnTransLeft(std::shared_ptr<ControlQueue> leftQueue);
void learnTransBody(std::shared_ptr<ControlQueue> leftQueue);
void learnFinalPush(std::shared_ptr<ControlQueue> leftQueue);

void executeRotHor90Deg(std::shared_ptr<ControlQueue> leftQueue);
void executeRotVert90Deg(std::shared_ptr<ControlQueue> leftQueue);
void executeRotHor180Deg(std::shared_ptr<ControlQueue> leftQueue);
void executeRotHor270Deg(std::shared_ptr<ControlQueue> leftQueue);
void executeTransRight(std::shared_ptr<ControlQueue> leftQueue);
void executeTransRight(std::shared_ptr<ControlQueue> leftQueue, double execTime);
void executeTransLeft(std::shared_ptr<ControlQueue> leftQueue);
void executeTransLeft(std::shared_ptr<ControlQueue> leftQueue, double execTime);
void executeTransBody(std::shared_ptr<ControlQueue> leftQueue);
void executeTransBody(std::shared_ptr<ControlQueue> leftQueue, double execTime);
void executeFinalPush(std::shared_ptr<ControlQueue> leftQueue);

int callClassifier(std::string pythonPath, std::string pythonFile, std::string pythonFun, std::string trainedPath, std::string passedFilePath);

shared_ptr<SensorData> dataRotHor;
shared_ptr<SensorData> dataRotVert;
shared_ptr<SensorData> dataTransRight;
shared_ptr<SensorData> dataTransLeft;
shared_ptr<SensorData> dataTransBody;
shared_ptr<SensorData> dataFinalPush;

arma::vec timesRotHor;
shared_ptr<JointDMPLearner> learnerRotHor;
std::shared_ptr<Dmp> dmpRotHor;

arma::vec timesRotVert;
shared_ptr<JointDMPLearner> learnerRotVert;
std::shared_ptr<Dmp> dmpRotVert;

arma::vec timesTransRight;
shared_ptr<JointDMPLearner> learnerTransRight;
std::shared_ptr<Dmp> dmpTransRight;

arma::vec timesTransLeft;
shared_ptr<JointDMPLearner> learnerTransLeft;
std::shared_ptr<Dmp> dmpTransLeft;

arma::vec timesTransBody;
shared_ptr<JointDMPLearner> learnerTransBody;
std::shared_ptr<Dmp> dmpTransBody;

arma::vec timesFinalPush;
shared_ptr<JointDMPLearner> learnerFinalPush;
std::shared_ptr<Dmp> dmpFinalPush;

double az;
double bz;
double tau;
double ac;
double dmpStepSize;
double tolAbsErr;
double tolRelErr;
double as;
double alpham;

int main(int argc, char** args) {

    int importanceSamplingCount;
    string inDir, cfFile, dataFolder, trajFile;
    vector<double> rlExploreSigmas;

    int seed = 100;
    std::mt19937* generator = new std::mt19937(seed);

    /*
    std::shared_ptr<NeverendingColorReward> trafficReward = nullptr;
    std::shared_ptr<ProjectiveSimulator> currentProjSim = nullptr;

    trafficReward = std::shared_ptr<NeverendingColorReward>(new NeverendingColorReward(generator, 2, 2, false));
    currentProjSim = std::shared_ptr<ProjectiveSimulator>(new ProjectiveSimulator(trafficReward, generator, 0.01, PS_USE_GEN, false));

    ofstream statFile; statFile.open("/home/c7031109/tmp/psgen.txt");
    PSEvaluator::produceStatistics(currentProjSim, trafficReward, 10, 0, NEVERENDINGCOLORREWARD_SUCCESSFUL_REWARD, statFile);
    currentProjSim->storePS("/home/c7031109/tmp/storedPS.txt");

    cout << "stored ps" << endl;

    std::shared_ptr<ProjectiveSimulator> loadedProjSim = std::shared_ptr<ProjectiveSimulator>(new ProjectiveSimulator(trafficReward, generator, "/home/c7031109/tmp/storedPS.txt"));

    cout << "loaded ps" << endl;

    loadedProjSim->storePS("/home/c7031109/tmp/storedPS2.txt");
    cout << "stored loaded ps" << endl;
    */

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

    cout << "1" << endl;

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    shared_ptr<ControlQueue> leftQueue = shared_ptr<ControlQueue>(new KukieControlQueue(kukaStepWaitTime, environment, "left_arm", *node));
    shared_ptr<ControlQueue> rightQueue = shared_ptr<ControlQueue>(new KukieControlQueue(kukaStepWaitTime, environment, "right_arm", *node));
cout << "2" << endl;
    shared_ptr<RosSchunk> leftHand = shared_ptr<RosSchunk>(new RosSchunk(*node, environment, "left"));
    shared_ptr<RosSchunk> rightHand = shared_ptr<RosSchunk>(new RosSchunk(*node, environment, "right"));
cout << "3" << endl;
    vector<shared_ptr<ControlQueue>> queueVectors;
    queueVectors.push_back(leftQueue);
cout << "4" << endl;
    learnFinalPush(leftQueue);
    learnRotHor(leftQueue);
    learnRotVert(leftQueue);
    learnTransBody(leftQueue);
    learnTransRight(leftQueue);
    learnTransLeft(leftQueue);
cout << "5" << endl;
    leftQueue->stopCurrentMode();
    leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
cout << "6" << endl;
    rightQueue->stopCurrentMode();
    rightQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
cout << "7" << endl;
    shared_ptr<thread> lqThread = leftQueue->startQueueThread();
    shared_ptr<thread> rqThread = rightQueue->startQueueThread();
cout << "8" << endl;
    int hapticCat = -1;

    while(hapticCat != 2) {

        vector<double> rightHandJoints = {0, -0.5238237461313833, 0.2120872918378427, 0.8655742259109377, 1.5389379959387146, -0.6260686922290597, 0.218843743489235877};
        rightHand->publishSdhJoints(rightHandJoints);

        vector<double> pushHandPos = {0.15, -1.57, 0, -1.57, 0, -1.57, 0};
        leftHand->publishSdhJoints(pushHandPos);

        goToStartPos(leftQueue);

        // get right blocking arm to position
        goToBlockingPos(rightQueue);

        hapticCat = doHapticTest(leftQueue, leftHand);
        cout << "(main) retrieved haptic category: " << hapticCat << endl;

        goToStartPos(leftQueue);

        leftQueue->stopCurrentMode();
        leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);

        // get right arm out of the way (DO NOT REMOVE THIS!!!)
        rightQueue->moveJoints(stdToArmadilloVec({0.864292, 1.54993, 0.872115, 1.77397, -2.81503, 0.464614, -1.73203}));

        leftHand->publishSdhJoints(pushHandPos);

        // do pushing stuff here
        if(hapticCat == 1) {
            // found bottom side --> rotate 270 degrees (later, execution time can be included in reward function of ps, which should find that counter clock wise rotation of 90 degrees is better)
            executeRotHor270Deg(leftQueue);
        } else if(hapticCat == 2) {
            // found closed side --> nothing to do; pick up
            break;
        } else if(hapticCat == 3) {
            // found open side --> rotate 180 degrees
            executeRotHor180Deg(leftQueue);
        } else if(hapticCat == 4) {
            // found top side --> rotate 90 degrees
            executeRotVert90Deg(leftQueue);
        }

        // move left arm away (DO NOT REMOVE THIS!!!)
        goToStartPos(leftQueue);

        // get right blocking arm to position
        goToBlockingPos(rightQueue);

        // do final push to blocking arm (HAS TO BE EXECUTED IN IMPEDANCE MODE)
        executeFinalPush(leftQueue);

    }

    // then do book peeling
    // ...

    leftQueue->stopCurrentMode();
    leftQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    goToStartPos(leftQueue);

    leftQueue->stopCurrentMode();
    leftQueue->setFinish();
    lqThread->join();

    rightQueue->stopCurrentMode();
    rightQueue->setFinish();
    rqThread->join();

    return 0;

}

int callClassifier(std::string pythonPath, std::string pythonFile, std::string pythonFun, std::string trainedPath, std::string passedFilePath) {

    int retVal = -1;
    string mName = pythonFile;
    string fName = pythonFun;
    string argumentVal = trainedPath;

    PyObject *pName, *pModule, *pFunc;
    PyObject *pArgs, *pValue;

    Py_Initialize();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString(string(string("sys.path.append('") + pythonPath + string("')")).c_str());
    PyRun_SimpleString("import trajlab_main");

    pName = PyUnicode_FromString(mName.c_str());
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {

        pFunc = PyObject_GetAttrString(pModule, fName.c_str());

        if (pFunc && PyCallable_Check(pFunc)) {

            pArgs = PyTuple_New(2);
            pValue = PyUnicode_FromString(argumentVal.c_str());
            if (!pValue) {
                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert argument\n");
                return 1;
            }
            PyTuple_SetItem(pArgs, 0, pValue);

            pValue = PyUnicode_FromString(passedFilePath.c_str());
            if (!pValue) {
                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert argument\n");
                return 1;
            }
            PyTuple_SetItem(pArgs, 1, pValue);

            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);
            if (pValue != NULL) {
                retVal = PyLong_AsLong(pValue);
                Py_DECREF(pValue);
            }
            else {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
                retVal = -1;
            }

        }
        else {
            if (PyErr_Occurred())
                PyErr_Print();
            cerr << "Cannot find function " << fName << endl;
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    }
    else {
        PyErr_Print();
        cerr << "Failed to load " << mName << endl;
        retVal = -1;
    }
    Py_Finalize();
    return retVal;

}

void goToBlockingPos(std::shared_ptr<ControlQueue> rightQueue) {

    rightQueue->moveJoints(stdToArmadilloVec({-2.3800294399261475, 1.5282957553863525, -2.280046224594116, 1.884490966796875, 2.1091063022613525, -1.4556314945220947, -0.7266652584075928}));

}

int doHapticTest(std::shared_ptr<ControlQueue> leftQueue, shared_ptr<RosSchunk> leftHand) {

    vector<shared_ptr<ControlQueue>> queues = {leftQueue};
    vector<shared_ptr<GenericHand>> hands = {leftHand};

    vector<double> handJoints = {0, -1.5, -1.0, 0, -0.2, -1.5, -1};
    leftHand->publishSdhJoints(handJoints);
    leftQueue->moveJoints(stdToArmadilloVec({-0.40275293588638306, 1.7016545534133911, 1.8671916723251343, -0.6587858200073242, 0.0556875579059124, 1.1993221044540405, -1.9818705320358276}));

    SensorStorage store(queues, hands, 100);
    shared_ptr<thread> storageThread = store.startDataStorage(tmpDataFolder + "hapticTest");

    sleep(1);

    vector<double> newHandJoints = {SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, 0.4, 1.2, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
    leftHand->publishSdhJoints(newHandJoints);

    sleep(1);

    store.stopDataStorage();
    storageThread->join();

    /*
    int classifierRes = callClassifier(resolvePath("$KUKADU_HOME/scripts/trajectory_classifier"),
                                       "trajlab_main", "runClassifier",
                                          resolvePath("$KUKADU_HOME/scripts/2015-05-11_data_with_labels/"),
                                          resolvePath(tmpDataFolder + "hapticTest/kuka_lwr_real_left_arm_0"));

    cout << "haptic test result is: " << classifierRes << endl;
    */

    /*
    int classifierRes;
    cout << "what was the haptic result?" << endl;
    cin >> classifierRes;
    */

    int classifierRes = 1;

    pf::remove_all(tmpDataFolder + "hapticTest");

    return classifierRes;

}

void goToStartPos(std::shared_ptr<ControlQueue> leftQueue) {

    leftQueue->moveJoints(stdToArmadilloVec({-1.1064038276672363, 1.6908159255981445, -0.8935614824295044, 1.4897072315216064, 1.2026464939117432, 1.549628734588623, 0.5572592616081238}));

}

void learnFinalPush(std::shared_ptr<ControlQueue> leftQueue) {
    cout << "a" << endl;
    dataFinalPush = SensorStorage::readStorage(leftQueue, pushDataFolder + "finalPush/kuka_lwr_real_left_arm_0");
    cout << "b" << endl;
    timesFinalPush = dataFinalPush->getTimes();
    cout << "c" << endl;
    learnerFinalPush = shared_ptr<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesFinalPush, dataFinalPush->getJointPos())));
    cout << "d" << endl;
    dmpFinalPush = learnerFinalPush->fitTrajectories();
    cout << "e" << endl;

}

void learnRotHor(std::shared_ptr<ControlQueue> leftQueue) {

    dataRotHor = SensorStorage::readStorage(leftQueue, pushDataFolder + "pushRot90CW_book_hor/kuka_lwr_real_left_arm_0");
    timesRotHor = dataRotHor->getTimes();
    learnerRotHor = shared_ptr<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesRotHor, dataRotHor->getJointPos())));
    dmpRotHor = learnerRotHor->fitTrajectories();

}

void learnRotVert(std::shared_ptr<ControlQueue> leftQueue) {

    dataRotVert = SensorStorage::readStorage(leftQueue, pushDataFolder + "pushRot90CW_book_vert/kuka_lwr_real_left_arm_0");
    timesRotVert = dataRotVert->getTimes();
    learnerRotVert = shared_ptr<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesRotVert, dataRotVert->getJointPos())));
    dmpRotVert = learnerRotVert->fitTrajectories();

}

void learnTransRight(std::shared_ptr<ControlQueue> leftQueue) {

    dataTransLeft = SensorStorage::readStorage(leftQueue, pushDataFolder + "pushTransRightToLeft/kuka_lwr_real_left_arm_0");
    timesTransLeft = dataTransLeft->getTimes();
    learnerTransLeft = shared_ptr<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesTransLeft, dataTransLeft->getJointPos())));
    dmpTransLeft = learnerTransLeft->fitTrajectories();

}

void learnTransLeft(std::shared_ptr<ControlQueue> leftQueue) {

    dataTransRight = SensorStorage::readStorage(leftQueue, pushDataFolder + "pushTransLeftToRight/kuka_lwr_real_left_arm_0");
    timesTransRight = dataTransRight->getTimes();
    learnerTransRight = shared_ptr<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesTransRight, dataTransRight->getJointPos())));
    dmpTransRight = learnerTransRight->fitTrajectories();

}

void learnTransBody(std::shared_ptr<ControlQueue> leftQueue) {

    dataTransBody = SensorStorage::readStorage(leftQueue, pushDataFolder + "pushTransTopToBody/kuka_lwr_real_left_arm_0");
    timesTransBody = dataTransBody->getTimes();
    learnerTransBody = shared_ptr<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesTransBody, dataTransBody->getJointPos())));
    dmpTransBody = learnerTransBody->fitTrajectories();

}

void executeFinalPush(std::shared_ptr<ControlQueue> leftQueue) {

    DMPExecutor execFinalPush(dmpFinalPush, leftQueue);
    execFinalPush.executeTrajectory(ac, 0, dmpFinalPush->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);

}

void executeRotHor90Deg(std::shared_ptr<ControlQueue> leftQueue) {

    DMPExecutor execRotHor(dmpRotHor, leftQueue);
    execRotHor.executeTrajectory(ac, 0, dmpRotHor->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);

}

void executeRotVert90Deg(std::shared_ptr<ControlQueue> leftQueue) {

    DMPExecutor execRotVert(dmpRotVert, leftQueue);
    execRotVert.executeTrajectory(ac, 0, dmpRotVert->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);

}

void executeRotHor180Deg(std::shared_ptr<ControlQueue> leftQueue) {

    executeRotHor90Deg(leftQueue);
    executeRotVert90Deg(leftQueue);

}

void executeRotHor270Deg(std::shared_ptr<ControlQueue> leftQueue) {

    executeRotHor180Deg(leftQueue);
    executeTransBody(leftQueue, 5.0);
    goToStartPos(leftQueue);
    executeRotHor90Deg(leftQueue);

}

void executeTransLeft(std::shared_ptr<ControlQueue> leftQueue) {

    executeTransLeft(leftQueue, dmpTransLeft->getTmax());

}

void executeTransLeft(std::shared_ptr<ControlQueue> leftQueue, double execTime) {

    DMPExecutor execTransLeft(dmpTransLeft, leftQueue);
    execTransLeft.executeTrajectory(ac, 0, execTime, dmpStepSize, tolAbsErr, tolRelErr);

}

void executeTransRight(std::shared_ptr<ControlQueue> leftQueue) {

    executeTransRight(leftQueue, dmpTransRight->getTmax());

}

void executeTransRight(std::shared_ptr<ControlQueue> leftQueue, double execTime) {

    DMPExecutor execTransRight(dmpTransRight, leftQueue);
    execTransRight.executeTrajectory(ac, 0, execTime, dmpStepSize, tolAbsErr, tolRelErr);

}

void executeTransBody(std::shared_ptr<ControlQueue> leftQueue) {

    executeTransBody(leftQueue, dmpTransBody->getTmax());

}

void executeTransBody(std::shared_ptr<ControlQueue> leftQueue, double execTime) {

    DMPExecutor execTransBody(dmpTransBody, leftQueue);
    execTransBody.executeTrajectory(ac, 0, execTime, dmpStepSize, tolAbsErr, tolRelErr);

}
