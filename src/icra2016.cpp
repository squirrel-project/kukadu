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

#define HAPTIC_MODE_CLASSIFIER 0
#define HAPTIC_MODE_TERMINAL 1

namespace po = boost::program_options;
namespace pf = boost::filesystem;

using namespace std;

int hapticMode = 1;

double az = 0.0;
double bz = 0.0;
double tau = 0.0;
double ac = 0.0;
double dmpStepSize = 0.0;
double tolAbsErr = 0.0;
double tolRelErr = 0.0;
double as = 0.0;
double alpham = 0.0;

string environment = "simulation";

int doHapticTest(std::shared_ptr<KukieControlQueue> leftQueue, shared_ptr<RosSchunk> leftHand, string classifierPath, string classifierFile, string classifierFunction, string databasePath, string tmpPath);

void goToStartPos(std::shared_ptr<ControlQueue> leftQueue);
void goToBlockingPos(std::shared_ptr<ControlQueue> rightQueue);

void learnRotHor(std::shared_ptr<ControlQueue> leftQueue, std::string trainingDataPath);
void learnRotVert(std::shared_ptr<ControlQueue> leftQueue, std::string trainingDataPath);
void learnFinalPush(std::shared_ptr<ControlQueue> leftQueue, std::string trainingDataPath);
void learnRotHorPushBack(std::shared_ptr<ControlQueue> leftQueue, std::string trainingDataPath);
void learnRotVertPushBack(std::shared_ptr<ControlQueue> leftQueue, std::string trainingDataPath);
void learnTransPushForward(std::shared_ptr<ControlQueue> rightQueue, std::string trainingDataPath);
void learnPick(std::shared_ptr<ControlQueue> leftQueue, std::string trainingDataPath);

void executeRotHor90Deg(std::shared_ptr<ControlQueue> leftQueue, std::shared_ptr<ControlQueue> rightQueue);
void executeRotVert90Deg(std::shared_ptr<ControlQueue> leftQueue, std::shared_ptr<ControlQueue> rightQueue);
void executeRotHor180Deg(std::shared_ptr<ControlQueue> leftQueue, std::shared_ptr<ControlQueue> rightQueue);
void executeRotVert180Deg(std::shared_ptr<ControlQueue> leftQueue, std::shared_ptr<ControlQueue> rightQueue);
void executeRotHor270Deg(std::shared_ptr<ControlQueue> leftQueue, std::shared_ptr<ControlQueue> rightQueue);
void executeRotVert270Deg(std::shared_ptr<ControlQueue> leftQueue, std::shared_ptr<ControlQueue> rightQueue);
void executeFinalPush(std::shared_ptr<ControlQueue> leftQueue);
void executeRotHorPushBack(std::shared_ptr<ControlQueue> leftQueue);
void executeRotVertPushBack(std::shared_ptr<ControlQueue> leftQueue);
void executeTransPushForward(std::shared_ptr<ControlQueue> rightQueue);
void executePick(std::shared_ptr<ControlQueue> leftQueue);

int callClassifier(std::string pythonPath, std::string pythonFile, std::string pythonFun, std::string trainedPath, std::string passedFilePath);

shared_ptr<SensorData> dataRotHor;
shared_ptr<SensorData> dataRotVert;
shared_ptr<SensorData> dataFinalPush;
shared_ptr<SensorData> dataPick;
shared_ptr<SensorData> dataRotHortPushBack;
shared_ptr<SensorData> dataRotVertPushBack;
shared_ptr<SensorData> dataTransPushForward;

arma::vec timesRotHortPushBack;
shared_ptr<JointDMPLearner> learnerRotHortPushBack;
std::shared_ptr<Dmp> dmpRotHortPushBack;

arma::vec timesRotVertPushBack;
shared_ptr<JointDMPLearner> learnerRotVertPushBack;
std::shared_ptr<Dmp> dmpRotVertPushBack;

arma::vec timesTransPushForward;
shared_ptr<JointDMPLearner> learnerTransPushForward;
std::shared_ptr<Dmp> dmpTransPushForward;

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

arma::vec timesPick;
shared_ptr<JointDMPLearner> learnerPick;
std::shared_ptr<Dmp> dmpPick;

int main(int argc, char** args) {

    int usePs = 0;
    int loadPs = 0;
    int simMode = 1;
    int numberOfActions = 0;
    int numberOfPercepts = 0;

    double gamma = 0.0;
    double stdReward = 0.0;

    std::mt19937* generator = new std::mt19937(std::chrono::system_clock::now().time_since_epoch().count());

    string psFile = "";
    string prepFinalPush = "";
    string prepRotHorPath = "";
    string prepRotVertPath = "";
    string pickTrajectoryPath = "";
    string prepRotHortPushBack = "";
    string prepRotVertPushBack = "";
    string prepTransPushForward = "";

    string tmpPath = "";
    string scriptPath = "";
    string scriptFile = "";
    string databasePath = "";
    string scriptFunction = "";


    shared_ptr<ManualReward> manualRew = nullptr;
    shared_ptr<ProjectiveSimulator> projSim = nullptr;

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
            ("pick.trajectory", po::value<string>(), "pick up trajectory")
            ("prep.rothor90deg", po::value<string>(), "horizontal cw rotation")
            ("prep.rotvert90deg", po::value<string>(), "vertical cw rotation")
            ("prep.rothor90degpushback", po::value<string>(), "translation push for horizontal rotation")
            ("prep.rotvert90degpushback", po::value<string>(), "translation push for vertical rotation")
            ("prep.transfinalpush", po::value<string>(), "final push")
            ("prep.transpushforward", po::value<string>(), "first push forward")
            ("ps.useprojectivesimulation", po::value<int>(), "use projective simulation or fixed mapping")
            ("ps.loadps", po::value<int>(), "load pre-trained projective simulator")
            ("ps.numberofactions", po::value<int>(), "number of actions")
            ("ps.numberofpercepts", po::value<int>(), "number of percepts")
            ("ps.gamma", po::value<double>(), "gamma of ps")
            ("ps.stdreward", po::value<double>(), "standard reward for successful execution")
            ("ps.psfile", po::value<string>(), "path to store and load the ps model")
            ("control.simulation", po::value<int>(), "use simulator")
            ("control.hapticmode", po::value<int>(), "mode for determining the haptic category")
            ("class.database", po::value<string>(), "classificatin database path")
            ("class.mmrpath", po::value<string>(), "classifier path")
            ("class.mmrfile", po::value<string>(), "classifier file")
            ("class.mmrfunction", po::value<string>(), "classifier function")
            ("class.tmppath", po::value<string>(), "temporary folder")
    ;

    ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/haptic.prop"), std::ifstream::in);
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
    if (vm.count("pick.trajectory")) pickTrajectoryPath = resolvePath(vm["pick.trajectory"].as<string>());
    else return 1;
    if (vm.count("prep.rothor90deg")) prepRotHorPath = resolvePath(vm["prep.rothor90deg"].as<string>());
    else return 1;
    if (vm.count("prep.rotvert90deg")) prepRotVertPath = resolvePath(vm["prep.rotvert90deg"].as<string>());
    else return 1;
    if (vm.count("prep.rothor90degpushback")) prepRotHortPushBack = resolvePath(vm["prep.rothor90degpushback"].as<string>());
    else return 1;
    if (vm.count("prep.rotvert90degpushback")) prepRotVertPushBack = resolvePath(vm["prep.rotvert90degpushback"].as<string>());
    else return 1;
    if (vm.count("prep.transpushforward")) prepTransPushForward = resolvePath(vm["prep.transpushforward"].as<string>());
    else return 1;
    if (vm.count("prep.transfinalpush")) prepFinalPush = resolvePath(vm["prep.transfinalpush"].as<string>());
    else return 1;
    if (vm.count("control.simulation")) simMode = vm["control.simulation"].as<int>();
    else return 1;
    if (vm.count("control.hapticmode")) hapticMode = vm["control.hapticmode"].as<int>();
    else return 1;
    if (vm.count("ps.useprojectivesimulation")) usePs = vm["ps.useprojectivesimulation"].as<int>();
    else return 1;
    if (vm.count("ps.loadps")) loadPs = vm["ps.loadps"].as<int>();
    else return 1;
    if (vm.count("ps.numberofactions")) numberOfActions = vm["ps.numberofactions"].as<int>();
    else return 1;
    if (vm.count("ps.numberofpercepts")) numberOfPercepts = vm["ps.numberofpercepts"].as<int>();
    else return 1;
    if (vm.count("ps.gamma")) gamma = vm["ps.gamma"].as<double>();
    else return 1;
    if (vm.count("ps.stdreward")) stdReward = vm["ps.stdreward"].as<double>();
    else return 1;
    if (vm.count("ps.psfile")) psFile = resolvePath(vm["ps.psfile"].as<string>());
    else return 1;
    if (vm.count("class.database")) databasePath = resolvePath(vm["class.database"].as<string>());
    else return 1;
    if (vm.count("class.mmrpath")) scriptPath = resolvePath(vm["class.mmrpath"].as<string>());
    else return 1;
    if (vm.count("class.mmrfile")) scriptFile = resolvePath(vm["class.mmrfile"].as<string>());
    else return 1;
    if (vm.count("class.mmrfunction")) scriptFunction = resolvePath(vm["class.mmrfunction"].as<string>());
    else return 1;
    if (vm.count("class.tmppath")) tmpPath = resolvePath(vm["class.tmppath"].as<string>());
    else return 1;

    environment = (simMode == 0) ? "real" : "simulation";

    pf::remove_all(tmpPath + "hapticTest");

    cout << "all properties loaded" << endl;
    cout << "execution mode is " << environment << endl;
    int kukaStepWaitTime = dmpStepSize * 1e6;

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    shared_ptr<KukieControlQueue> leftQueue = shared_ptr<KukieControlQueue>(new KukieControlQueue(kukaStepWaitTime, environment, "left_arm", *node));
    shared_ptr<KukieControlQueue> rightQueue = shared_ptr<KukieControlQueue>(new KukieControlQueue(kukaStepWaitTime, environment, "right_arm", *node));

    shared_ptr<RosSchunk> leftHand = shared_ptr<RosSchunk>(new RosSchunk(*node, environment, "left"));
    shared_ptr<RosSchunk> rightHand = shared_ptr<RosSchunk>(new RosSchunk(*node, environment, "right"));

    vector<double> pushHandPos = {0.15, -1.57, 0, -1.57, 0, -1.57, 0};
    leftHand->publishSdhJoints(pushHandPos);

    vector<shared_ptr<ControlQueue>> queueVectors;
    queueVectors.push_back(leftQueue);

    if(usePs) {
        manualRew = shared_ptr<ManualReward>(new ManualReward(generator, numberOfActions, numberOfPercepts, false, stdReward));

        if(!loadPs)
            projSim = shared_ptr<ProjectiveSimulator>(new ProjectiveSimulator(manualRew, generator, gamma, PS_USE_ORIGINAL, false));
        else
            projSim = shared_ptr<ProjectiveSimulator>(new ProjectiveSimulator(manualRew, generator, psFile));
    }

    if(simMode) {

        // add sponge
        shared_ptr<SimInterface> simInterface = shared_ptr<SimInterface>(new SimInterface (argc, args, kukaStepWaitTime, *node));
        simInterface->addPrimShape(1, "sponge", {0.3, 0.7, 0.01}, {0, 0, 0, 0}, {0.8, 2, 0.075}, 10.0);
        simInterface->setObjMaterial("sponge", SimInterface::FRICTION_HIGH);

        ros::Rate r(1);
        r.sleep();

        // add book
    //    simInterface->addPrimShape(1, "book", {0.15, 0.72, 0.2}, {0, 0, 0, 0}, {0.25, 0.17, 0.04}, 10.0);
    //    simInterface->setObjMaterial("book", SimInterface::FRICTION_NO);

    }

    learnRotHor(leftQueue, prepRotHorPath);
    learnRotVert(leftQueue, prepRotVertPath);
    learnFinalPush(leftQueue, prepFinalPush);
    learnRotHorPushBack(leftQueue, prepRotHortPushBack);
    learnRotVertPushBack(leftQueue, prepRotVertPushBack);
    learnTransPushForward(rightQueue, prepTransPushForward);
    learnPick(leftQueue, pickTrajectoryPath);

    leftQueue->stopCurrentMode();
    leftQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);

    rightQueue->stopCurrentMode();
    rightQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);

    shared_ptr<thread> lqThread = leftQueue->startQueueThread();
    shared_ptr<thread> rqThread = rightQueue->startQueueThread();

    int performFurtherRound = -1;

    while(performFurtherRound) {

        vector<double> rightHandJoints = {2.57271414799788e-05, -0.5270757987175063, 0.20988086261097294, 0.8659628476930076, 1.537404015150829, -0.5575713234648674, 0.22337989354436658};
        rightHand->publishSdhJoints(rightHandJoints);

        vector<double> pushHandPos = {0.15, -1.57, 0, -1.57, 0, -1.57, 0};
        leftHand->publishSdhJoints(pushHandPos);

        cout << "(main) moving left arm to starting position" << endl;
        goToStartPos(leftQueue);

        // get right blocking arm to position
        cout << "(main) moving right arm to blocking position" << endl;
        goToBlockingPos(rightQueue);

        cout << "(main) do haptic test" << endl;
        // - 1 because classifier returns values in [1, 4]
        int hapticCat = doHapticTest(leftQueue, leftHand, scriptPath, scriptFile, scriptFunction, databasePath, tmpPath) - 1;
        cout << "(main) retrieved haptic category: " << (hapticCat + 1) << endl;

        cout << "(main) moving left arm to starting position" << endl;
        goToStartPos(leftQueue);

        cout << "(main) moving left arm to pushing initial position" << endl;
        leftHand->publishSdhJoints(pushHandPos);

        int nextActionId = 0;
        if(!usePs) {

            // fixed mapping between haptic categories and preparation controllers
            // next action id = haptic cat (action ids are ordered such that they match the haptic categories)
            nextActionId = hapticCat;

        } else {

            // use projective simulation to learn mapping between haptic categories and preparation controllers
            // haptic category is input percept
            manualRew->setNextPerceptId(hapticCat);
            shared_ptr<ActionClip> nextAction = projSim->performRandomWalk();
            nextActionId = nextAction->getActionId();

        }

        // do pushing stuff here
        cout << "(main) performing preparation action " << (nextActionId + 1) << endl;
        if(nextActionId == 0) {
            // found bottom side --> rotate 270 degrees (later, execution time can be included in reward function of ps, which should find that counter clock wise rotation of 90 degrees is better)
            executeRotVert270Deg(leftQueue, rightQueue);
        } else if(nextActionId == 1) {
            // found closed side --> nothing to do
        } else if(nextActionId == 2) {
            // found open side --> rotate 180 degrees
            executeRotHor180Deg(leftQueue, rightQueue);
        } else if(nextActionId == 3) {
            // found top side --> rotate 90 degrees
            executeRotVert90Deg(leftQueue, rightQueue);
        }

        /*
        // then do book peeling
        cout << "(main) try to pick up book now" << endl;
        executePick(leftQueue);
        */

        if(usePs) {
            projSim->performRewarding();
        }

        // move left arm away (DO NOT REMOVE THIS!!!)
        cout << "(main) moving left arm back to starting position" << endl;
        goToStartPos(leftQueue);

        // get right blocking arm to position
        cout << "(main) moving right arm back to starting position" << endl;
        goToBlockingPos(rightQueue);

        cout << "(main) want to do one more round of training? (0 = no / 1 = yes)" << endl;
        cin >> performFurtherRound;

    }

    cout << "(main) stop further training" << endl;

    if(usePs) {
        cout << "(main) storing trained projective simulator" << endl;
        projSim->storePS(psFile);
    }

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

int doHapticTest(std::shared_ptr<KukieControlQueue> leftQueue, shared_ptr<RosSchunk> leftHand, string classifierPath, string classifierFile, string classifierFunction, string databasePath, string tmpPath) {

    int classifierRes = -1;
    vector<shared_ptr<ControlQueue>> queues = {leftQueue};
    vector<shared_ptr<GenericHand>> hands = {leftHand};

    vector<double> handJoints = {0, -1.5, -1.0, 0, -0.2, -1.5, -1};
    leftHand->publishSdhJoints(handJoints);
    leftQueue->moveJoints(stdToArmadilloVec({-0.40275293588638306, 1.7016545534133911, 1.8671916723251343, -0.6587858200073242, 0.0556875579059124, 1.1993221044540405, -1.9818705320358276}));

    leftQueue->stopCurrentMode();
    leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);

    SensorStorage store(queues, hands, 100);
    shared_ptr<thread> storageThread = store.startDataStorage(tmpPath + "hapticTest");

    sleep(1);

    vector<double> newHandJoints = {SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, 0.4, 1.2, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
    leftHand->publishSdhJoints(newHandJoints);

    sleep(2);

    store.stopDataStorage();
    storageThread->join();

    if(hapticMode == HAPTIC_MODE_TERMINAL) {
        cout << "what was the haptic result?" << endl;
        cin >> classifierRes;
    } else if(hapticMode == HAPTIC_MODE_CLASSIFIER) {
        classifierRes = callClassifier(classifierPath,
                                               classifierFile, classifierFunction,
                                                  databasePath,
                                                  tmpPath + "hapticTest/kuka_lwr_" + leftQueue->getRobotDeviceType() + "_left_arm_0");
    } else {
        throw "haptic mode not known";
    }

    getchar();

    leftQueue->stopCurrentMode();
    leftQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);

    cout << "(main) classifier result is category " << classifierRes << endl;

    pf::remove_all(tmpPath + "hapticTest");
    leftHand->publishSdhJoints(handJoints);

    return classifierRes;

}

void goToStartPos(std::shared_ptr<ControlQueue> leftQueue) {

    leftQueue->moveJoints(stdToArmadilloVec({-1.1064038276672363, 1.6908159255981445, -0.8935614824295044, 1.4897072315216064, 1.2026464939117432, 1.549628734588623, 0.5572592616081238}));

}

void learnPick(std::shared_ptr<ControlQueue> leftQueue, std::string trainingDataPath) {

    dataPick = SensorStorage::readStorage(leftQueue, trainingDataPath);
    timesPick = dataFinalPush->getTimes();
    learnerPick = shared_ptr<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesFinalPush, dataFinalPush->getJointPos())));
    dmpPick = learnerFinalPush->fitTrajectories();

}

void learnFinalPush(std::shared_ptr<ControlQueue> leftQueue, string trainingDataPath) {

    dataFinalPush = SensorStorage::readStorage(leftQueue, trainingDataPath);
    timesFinalPush = dataFinalPush->getTimes();
    learnerFinalPush = shared_ptr<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesFinalPush, dataFinalPush->getJointPos())));
    dmpFinalPush = learnerFinalPush->fitTrajectories();

}

void learnRotHor(std::shared_ptr<ControlQueue> leftQueue, string trainingDataPath) {

    dataRotHor = SensorStorage::readStorage(leftQueue, trainingDataPath);
    timesRotHor = dataRotHor->getTimes();
    learnerRotHor = shared_ptr<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesRotHor, dataRotHor->getJointPos())));
    dmpRotHor = learnerRotHor->fitTrajectories();

}

void learnRotVert(std::shared_ptr<ControlQueue> leftQueue, string trainingDataPath) {

    dataRotVert = SensorStorage::readStorage(leftQueue, trainingDataPath);
    timesRotVert = dataRotVert->getTimes();
    learnerRotVert = shared_ptr<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesRotVert, dataRotVert->getJointPos())));
    dmpRotVert = learnerRotVert->fitTrajectories();

}

void learnRotHorPushBack(std::shared_ptr<ControlQueue> leftQueue, std::string trainingDataPath) {

    dataRotHortPushBack = SensorStorage::readStorage(leftQueue, trainingDataPath);
    timesRotHortPushBack = dataRotHortPushBack->getTimes();
    learnerRotHortPushBack = shared_ptr<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesRotHortPushBack, dataRotHortPushBack->getJointPos())));
    dmpRotHortPushBack = learnerRotHortPushBack->fitTrajectories();

}

void learnRotVertPushBack(std::shared_ptr<ControlQueue> leftQueue, std::string trainingDataPath) {

    dataRotVertPushBack = SensorStorage::readStorage(leftQueue, trainingDataPath);
    timesRotVertPushBack = dataRotVertPushBack->getTimes();
    learnerRotVertPushBack = shared_ptr<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesRotVertPushBack, dataRotVertPushBack->getJointPos())));
    dmpRotVertPushBack = learnerRotVertPushBack->fitTrajectories();

}

void learnTransPushForward(std::shared_ptr<ControlQueue> rightQueue, std::string trainingDataPath) {

    dataTransPushForward = SensorStorage::readStorage(rightQueue, trainingDataPath);
    timesTransPushForward = dataTransPushForward->getTimes();
    learnerTransPushForward = shared_ptr<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesTransPushForward, dataTransPushForward->getJointPos())));
    dmpTransPushForward = learnerTransPushForward->fitTrajectories();

}

void executePick(std::shared_ptr<ControlQueue> leftQueue) {

    DMPExecutor execPick(dmpPick, leftQueue);
    execPick.executeTrajectory(ac, 0, dmpPick->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);

}

void executeFinalPush(std::shared_ptr<ControlQueue> leftQueue) {

    leftQueue->moveJoints({-0.9692263007164001, 1.113829493522644, 1.1473214626312256, -1.444376826286316, -0.28663957118988037, -0.8957559466362, -0.2651996612548828});

    leftQueue->stopCurrentMode();
    leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);

    DMPExecutor execFinalPush(dmpFinalPush, leftQueue);
    execFinalPush.executeTrajectory(ac, 0, dmpFinalPush->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);

    leftQueue->stopCurrentMode();
    leftQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);

    leftQueue->moveJoints({-0.9692263007164001, 1.113829493522644, 1.1473214626312256, -1.444376826286316, -0.28663957118988037, -0.8957559466362, -0.2651996612548828});

}

void executeRotHorPushBack(std::shared_ptr<ControlQueue> leftQueue) {

    leftQueue->moveJoints({-0.26015156507492065, 1.7727789878845215, 2.0502824783325195, -0.4911971092224121, -1.933821678161621, 1.159960150718689, 0.3724197447299957});

    DMPExecutor execRotHorPushBack(dmpRotHortPushBack, leftQueue);
    execRotHorPushBack.executeTrajectory(ac, 0, dmpRotHortPushBack->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);

    leftQueue->moveJoints({-0.26015156507492065, 1.7727789878845215, 2.0502824783325195, -0.4911971092224121, -1.933821678161621, 1.159960150718689, 0.3724197447299957});

}

void executeRotVertPushBack(std::shared_ptr<ControlQueue> leftQueue) {

    DMPExecutor execRotVertPushBack(dmpRotVertPushBack, leftQueue);
    execRotVertPushBack.executeTrajectory(ac, 0, dmpRotVertPushBack->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);

}

void executeTransPushForward(std::shared_ptr<ControlQueue> rightQueue) {

    DMPExecutor execTransPushForward(dmpTransPushForward, rightQueue);
    execTransPushForward.executeTrajectory(ac, 0, dmpTransPushForward->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);

    goToBlockingPos(rightQueue);

}

void executeRotHor90Deg(std::shared_ptr<ControlQueue> leftQueue, std::shared_ptr<ControlQueue> rightQueue) {

    executeTransPushForward(rightQueue);

    leftQueue->moveJoints({-1.2000699043273926, 1.2644866704940796, 2.038100004196167, -2.071664810180664, -1.1690874099731445, 1.3248648643493652, -1.7968707084655762});

    DMPExecutor execRotHor(dmpRotHor, leftQueue);
    execRotHor.executeTrajectory(ac, 0, dmpRotHor->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);

    executeRotHorPushBack(leftQueue);
    executeFinalPush(leftQueue);

}

void executeRotVert90Deg(std::shared_ptr<ControlQueue> leftQueue, std::shared_ptr<ControlQueue> rightQueue) {

    executeTransPushForward(rightQueue);

    DMPExecutor execRotVert(dmpRotVert, leftQueue);
    execRotVert.executeTrajectory(ac, 0, dmpRotVert->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);

    executeRotVertPushBack(leftQueue);
    executeFinalPush(leftQueue);

}

void executeRotHor180Deg(std::shared_ptr<ControlQueue> leftQueue, std::shared_ptr<ControlQueue> rightQueue) {

    executeRotHor90Deg(leftQueue, rightQueue);
    goToStartPos(leftQueue);
    executeRotVert90Deg(leftQueue, rightQueue);

}

void executeRotVert180Deg(std::shared_ptr<ControlQueue> leftQueue, std::shared_ptr<ControlQueue> rightQueue) {

    executeRotVert90Deg(leftQueue, rightQueue);
    goToStartPos(leftQueue);
    executeRotHor90Deg(leftQueue, rightQueue);

}

void executeRotHor270Deg(std::shared_ptr<ControlQueue> leftQueue, std::shared_ptr<ControlQueue> rightQueue) {

    executeRotHor180Deg(leftQueue, rightQueue);
    goToStartPos(leftQueue);
    executeRotHor90Deg(leftQueue, rightQueue);

}

void executeRotVert270Deg(std::shared_ptr<ControlQueue> leftQueue, std::shared_ptr<ControlQueue> rightQueue) {

    executeRotVert180Deg(leftQueue, rightQueue);
    goToStartPos(leftQueue);
    executeRotVert90Deg(leftQueue, rightQueue);

}
