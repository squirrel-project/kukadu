
#include <cstdio>
#include <iostream>
#include <fstream>
#include <thread>
#include <string>
#include <vector>
#include <wordexp.h>
#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include "../include/kukadu.h"
#include "utils/utils.h"


using namespace std;
using namespace arma;

//int test=1; //testing trajectory without dmps
//int test=2; //testing trajectory with dmps
int test = 3;


std::string hand = "left";
std::string arm = "left_arm";
std::string environment = "simulation";
int doSimulation = 1;

bool record = false;

//string hand="right";
//string arm="right_arm";


std::string resolvePath(std::string path);
geometry_msgs::Pose vectordouble2pose(std::vector<double>* vectorpose);
geometry_msgs::Pose vectorarma2pose(arma::vec* vectorpose);
arma::mat readMovements(std::string file);

void executeTrajCart(std::shared_ptr<OrocosControlQueue> movementQu, string file);
void collectDataCart(OrocosControlQueue* queue, SimInterface* sim, string object_id);
t_executor_res executeDMPcart(OrocosControlQueue* movementQu, string file, string file1, int doSimulation, double az, double bz, int plotResults, SimInterface* simI,string object_id);
t_executor_res executePushCart(OrocosControlQueue* movementQu, string file, string file1, int doSimulation, double az, double bz, int plotResults, SimInterface* simI,string object_id);
t_executor_res executeDemoPush(shared_ptr<OrocosControlQueue> movementQu,shared_ptr<SimInterface> SimI, string file,string fileObject, double az, double bz, int plotResults, int doSimulation, string object_id);

void collectDataSim(std::shared_ptr<OrocosControlQueue> queue,std::shared_ptr<SimInterface> sim, string objectId, string fileR, string fileO, string fileS);
double az = 48.0;
double bz = (az - 1) / 4;

int temp = 0;


int kukaStepWaitTime = 14 * 1e4;
double dmpStepSize = kukaStepWaitTime * 1e-6;


int main(int argc, char** args) {

    cout << "gsm start; creating interfaces" << endl;

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);


    std::shared_ptr<OrocosControlQueue> queue = std::shared_ptr<OrocosControlQueue>(new OrocosControlQueue(kukaStepWaitTime, environment, arm, *node));
    cout << "control queue interface created" << endl;

    RosSchunk* handQ = new RosSchunk(*node, environment, hand);
    cout << "hand interface created" << endl;

    //hand
    vector<double> newPos = {0, -1.57, 0, -1.57,0,-1.57,0};
    handQ->publishSdhJoints(newPos);

    //arm

    queue->stopCurrentMode();
    queue->switchMode(10);
    queue->startQueueThread();
    cout << "queue started" << endl;

    cout << "moving to start position" << endl;

    vector<double> newPoC1 = { 0.272241000000000,	0.876041000000000,	0.395563000000000,	0.776795000000000,	0.0112587000000000,	-0.0194455000000000,	0.629462000000000};
    queue->moveCartesian(vectordouble2pose(&newPoC1));

    //simulator
    std::shared_ptr<SimInterface> simI= std::shared_ptr<SimInterface>(new SimInterface (argc, args, kukaStepWaitTime, *node));
    cout << "sim interface created importing objects" << endl;

    float newPs[3] = {0.5, 0.7, 0.01};
    float newOs[4] = {0, 0, 0, 0};
    float dims[3] = {1, 2, 0.08};

    simI->addPrimShape(1, "sponge", newPs, newOs, dims, 20.0);
    simI->setObjMaterial("sponge","lowFrictionMaterial");

    sleep(1);

    //taught data

    cout << "loading taught data " << endl;

    string fileO = "/home/c7031098/testing/testCar1/SimposO";
    string fileR = "/home/c7031098/testing/dataCart/car1";
    string fileS = "/home/c7031098/testing/testCar1/SimPosS";

    string fileDMP="/home/c7031098/testing/dataCart/car1.txt";

    vector<vector<double>> Ex;
    vector<vector<double>> Ey;
    vector<vector<double>> sigma;
    vector<string> trajR;
    vector<string> trajO;
    vector<Dmp> dmpLib;

    mat trajecO = readMovements(fileO + ".txt");

    //cout << "rows "<< trajecO.n_rows<< endl;
    //cout << "cols "<< trajecO.n_cols<< endl;


    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> sig;

    for (int i = 0; i < trajecO.n_rows; i++){
        x.push_back(trajecO(i,1));
        y.push_back(trajecO(i,2));
        sig.push_back(0.0);
    }

    Ex.push_back(x);
    Ey.push_back(y);
    sigma.push_back(sig);
    trajR.push_back(fileR);
    trajO.push_back(fileO);

    cout << "initial vectors created" << endl;


    //dmps base

    vector<double> tmpmys{0, 1, 2, 3, 4, 4.1, 4.3, 4.5};
    vector<double> tmpsigmas{0.2, 0.8};

    // reading in file
    mat carts = readMovements(fileR +".txt");
    tmpmys = constructDmpMys(carts);

    double tau = 0.8;
    double ax = -log((float)0.1) / carts(carts.n_rows - 1, 0) / tau;

    vector<DMPBase> baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);
    TrajectoryDMPLearner dmpLearner(baseDef, tau, az, bz, ax, carts);
    Dmp learnedDmps = dmpLearner.fitTrajectories();

    dmpLib.push_back(learnedDmps);

    cout << "created dmp and stored" << endl;


    float newP[3];
    float newO[4];
    float dim[3];
    int shape;
    float weight;
    string objectId;
    int classO;

    int n = 1; //number of experiments

    for (int i = 0; i < n; i++){


        cout << "Performing experiment " << i << endl;
        cout << "Going to start position " << i << endl;
        queue->moveCartesian(vectordouble2pose(&newPoC1));

        simI->setObjPose("sponge", newPs, newOs);

        if (i == 0){
            newP[0] = 0.30; newP[1] = 0.7, newP[2] = 0.2;
            newO[0] = 0.0; newO[1] = 0.0; newO[2] = 0.0; newO[3] = 0.0;
            dim[0] = 0.2; dim[1] = 0.2; dim[2] = 0.1;
            shape = 1;
            weight = 1.0;
            objectId = "box";
            classO= 0;
        }



        simI->addPrimShape(shape, objectId, newP, newO, dim, weight);
        cout << " object imported " << objectId << endl;

        arma::vec startP=dmpLib.at(classO).getY0();

        cout << "start position " << endl;

        startP[3] =	0.7767950; startP[4] =	0.0112587000; startP[5] = -0.019445500; startP[6] =	0.62946200;

        try {queue->moveCartesian(vectorarma2pose(&startP));}
        catch (...) { cout << "exception cart movement " << startP<< endl;}
        queue->stopCurrentMode();
        queue->switchMode(20);

        std::thread* inputThrC = NULL;
        record = true;
        inputThrC = new std::thread(collectDataSim, queue, simI,  objectId, fileR + std::to_string(i) + ".txt" ,  fileO + std::to_string(i) + ".txt", fileS +  std::to_string(i) + ".txt");
        cout << "record started " << endl;

        DMPExecutorPush dmpexec(dmpLib, queue, doSimulation, simI, objectId);
        t_executor_res dmpResult = dmpexec.simulateTrajectory(2);

        //DMPExecutor dmpexec(dmpLib.at(0), queue, 0);
        //t_executor_res dmpResult = dmpexec.simulateTrajectory(2);


        record = false;



        //t_executor_res demoRes= executeDemoPush(queue,simI, fileR ,fileO,  az,  bz, 0, 1, objectId);
        simI->removeObj(objectId);

    }

    getchar();

    return 0;
}



/*t_executor_res executeDemoPush(shared_ptr<OrocosControlQueue> movementQu,shared_ptr<SimInterface> SimI, string file,string fileObject, double az, double bz, int plotResults, int doSimulation, string object_id) {

    Gnuplot* g1 = NULL;

    vector<double> tmpmys{0, 1, 2, 3, 4, 4.1, 4.3, 4.5};
    //vector<double> tmpmys;
    vector<double> tmpsigmas{0.2, 0.8};

    // reading in file
    mat carts = readMovements(file);
    tmpmys = constructDmpMys(carts);
    mat desPos= readMovements(fileObject);

    double tau = 0.8;
    double ax = -log((float)0.1) / carts(carts.n_rows - 1, 0) / tau;

    vector<DMPBase> baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);

    TrajectoryDMPLearner dmpLearner(baseDef, tau, az, bz, ax, carts);
    Dmp learnedDmps = dmpLearner.fitTrajectories();
    arma::vec startP=learnedDmps.getY0();

    movementQu->moveCartesian(vectorarma2pose(&startP));
    movementQu->stopCurrentMode();
    movementQu->switchMode(20);

    DMPExecutorPush dmpexec(learnedDmps, movementQu, doSimulation, SimI, object_id);
    t_executor_res dmpResult = dmpexec.simulateTrajectory(2);

    int plotNum = learnedDmps.getDegreesOfFreedom();

    if(plotResults) {

        for(int plotTraj = 0; plotTraj < plotNum; ++plotTraj) {

            ostringstream convert;   // stream used for the conversion
            convert << plotTraj;

            string title = string("fitted sensor data (joint") + convert.str() + string(")");
            g1 = new Gnuplot(title);
            g1->set_style("lines").plot_xy(armadilloToStdVec(learnedDmps.getSupervisedTs()), armadilloToStdVec(learnedDmps.getSampleYByIndex(plotTraj)), "sample y");
            g1->set_style("lines").plot_xy(armadilloToStdVec(dmpResult.t), armadilloToStdVec(dmpResult.y[plotTraj]), "dmp y");
            g1->showonscreen();

        }

    }

    return dmpResult;

}*/


geometry_msgs::Pose vectordouble2pose(std::vector<double>* vectorpose){

    geometry_msgs:: Pose posepose;
    posepose.position.x = vectorpose->at(0);
    posepose.position.y = vectorpose->at(1);
    posepose.position.z = vectorpose->at(2);
    posepose.orientation.x = vectorpose->at(3);
    posepose.orientation.y = vectorpose->at(4);
    posepose.orientation.z = vectorpose->at(5);
    posepose.orientation.w = vectorpose->at(6);

    return posepose;

}

geometry_msgs::Pose vectorarma2pose(arma::vec* vectorpose){

    geometry_msgs:: Pose posepose;
    posepose.position.x = vectorpose->at(0);
    posepose.position.y = vectorpose->at(1);
    posepose.position.z = vectorpose->at(2);
    posepose.orientation.x = vectorpose->at(3);
    posepose.orientation.y = vectorpose->at(4);
    posepose.orientation.z = vectorpose->at(5);
    posepose.orientation.w = vectorpose->at(6);

    return posepose;

}

void executeTrajCart(std::shared_ptr<OrocosControlQueue> movementQu, string file){

    int columns = 8;
    mat coord = readMovements(file);
    int duration = coord.n_rows;
    movementQu->stopCurrentMode();
    movementQu->switchMode(20);
    cout<<"starting movement"<<endl;
    sleep(0.5);

    for (int i =0; i < duration; i=i+1) {
        vector<double> moveCarts;
        for(int j = 0; j < 7; ++j){
            sleep(0.5);
            cout<<coord(i,j+1)<<" ";
            moveCarts.push_back(coord(i,j+1));

        }
        cout<<endl;

        // vector<double> diff={0.0,	0.00,	0.05,	0.0,	0.0,	0.0,	1.0};
        vector<double> diff={0.0,	0.00,	0.0,	0.0,	0.0,	0.0,	1.0};
        movementQu->moveCartesianRelativeWf(vectordouble2pose(&moveCarts), vectordouble2pose(&diff));

    }

}

void collectDataSim(std::shared_ptr<OrocosControlQueue> queue,std::shared_ptr<SimInterface> sim, string objectId, string fileR, string fileO, string fileS){

    ofstream oFile;
    ofstream sFile;
    ofstream rFile;

    oFile.open(fileO);
    sFile.open(fileS);
    rFile.open(fileR);

    cout << "files open" << endl;

    double time = 0.0;
    double lastTime = -1.0;
    arma::vec wrench;
    int columns = 8;

    double start= ros::Time::now().toSec();

    while(record){

        mes_result mesResJ = queue->getCurrentJoints();
        geometry_msgs::Pose PoseR = queue->getCartesianPose();
        geometry_msgs::Pose Pose = sim->getObjPose(objectId);
        mes_result mesResS = queue->getCurrentCartesianFrcTrq();

        time = ros::Time::now().toSec() - start;
        wrench = mesResS.joints;


        usleep(0.5 * 1e4);
        if(lastTime != time) {

            rFile << time;
            rFile << time;
            rFile << "\t" << PoseR.position.x;
            rFile << "\t" << PoseR.position.y;
            rFile << "\t" << PoseR.position.z;
            rFile << "\t" << PoseR.orientation.x;
            rFile << "\t" << PoseR.orientation.y;
            rFile << "\t" << PoseR.orientation.z;
            rFile << "\t" << PoseR.orientation.w;
            rFile << endl;

            sFile << time;
            for(int i = 0; i < columns - 1; ++i) { sFile << "\t" << wrench[i]; }
            sFile << endl;

            oFile << time;
            oFile << "\t" << Pose.position.x;
            oFile << "\t" << Pose.position.y;
            oFile << "\t" << Pose.position.z;
            oFile << "\t" << Pose.orientation.x;
            oFile << "\t" << Pose.orientation.y;
            oFile << "\t" << Pose.orientation.z;
            oFile << "\t" << Pose.orientation.w;
            oFile << endl;

            lastTime = time;

        }
    }




}



