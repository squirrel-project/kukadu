#include <cstdio>
#include <iostream>
#include <fstream>
#include <thread>

#include <wordexp.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include "../include/kukadu.h"

#include "std_msgs/Float64MultiArray.h"
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>

#define DOSIMULATION 1

using namespace std;
using namespace arma;

int experiment = 3;
//int experiment = 1; // collectiong data by kinesthetic teaching
//int experiment = 2; // reproducing movement in real world and collectiong data
//int experiment = 3; // reproducing movement in simulator and collectiong data

bool record = false;
bool start = false;
bool firstSet = false;
int head = 0;

int kukaStepWaitTime = 14 * 1e4;
float pickMaxAxisTorque = 2.0;
double az = 48.0;
double bz = (az - 1) / 4;

char consoleInputter();
char consoleInput = 0;

std::string hand = "left";
std::string arm = "left_arm";
std::string environment = "simulation";
string headTopic = "/"+environment+"/"+"kit_head/joint_control/move";


void collectDataReal(std::shared_ptr<OrocosControlQueue> queue, string fileR, string fileO, string fileS);
void collectDataSim(std::shared_ptr<OrocosControlQueue> queue,std::shared_ptr<SimInterface> sim, string objecId, string fileR, string fileO, string fileS);
geometry_msgs::Pose vectordouble2pose(std::vector<double>* vectorpose);
t_executor_res executePush(shared_ptr<OrocosControlQueue> movementQu, string file, double az, double bz);


ros::Publisher headMove;
tf::tfMessage newTarget;

geometry_msgs::Vector3 translation;
geometry_msgs::Quaternion rotation;

void arCallback(tf::tfMessage msg);

int main(int argc, char** args) {


    string fileO = "/home/c7031098/testing/testCar1/SimposO1.txt";
    string fileS = "/home/c7031098/testing/testCar1/SimPosS1.txt";
    string fileR = "/home/c7031098/testing/testCar1/SimPosR1.txt";

    //string fileDMP = "/home/c7031098/testing/testCar1/posR.txt";

    string fileDMP = "/home/c7031098/testing/testCar1/car1.txt";


    ros::init(argc, args, "push");

    ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);

    std::shared_ptr<OrocosControlQueue> queue = std::shared_ptr<OrocosControlQueue>(new OrocosControlQueue(kukaStepWaitTime, environment, arm, *node));
    std::shared_ptr<std::thread> raThr = std::shared_ptr<std::thread>(nullptr);

    queue = std::shared_ptr<OrocosControlQueue>(new OrocosControlQueue(kukaStepWaitTime, environment, arm, *node));

    RosSchunk* handQ=new RosSchunk(*node, environment, hand);
    cout<<"hand interface created"<<endl;

    //moving hand to staring position
    vector<double> newPos = {0, -1.57, 0, -1.57, 0, -1.57, 0};
    handQ->publishSdhJoints(newPos);


    cout<< "initilization done"<<endl;

    switch(experiment){

    case 1:{

        ofstream oFile;
        ofstream sFile;
        ofstream rFile;

        oFile.open(fileO);
        sFile.open(fileS);
        rFile.open(fileR);

        double time = 0.0;
        double lastTime = -1.0;
        arma::vec wrench;
        int columns = 8;

        queue->stopCurrentMode();
        raThr = queue->startQueueThread();
        queue->setStiffness(0.2, 0.01, 0.2, 15000, 150, pickMaxAxisTorque);

        queue->switchMode(30);

        std::thread* inputThr = NULL;
        inputThr = new std::thread(consoleInputter);

        while(consoleInput == 0) {

            geometry_msgs::Pose PoseR = queue->getCartesianPose();
            mes_result mesResS = queue->getCurrentCartesianFrcTrq();

            time = mesResS.time;
            wrench = mesResS.joints;

            usleep(0.5 * 1e4);
            if(wrench.n_elem > 1 && lastTime != time) {

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

                lastTime = time;

            }

        }

        queue->stopCurrentMode();
        queue->switchMode(10);
        queue->stopCurrentMode();

        break;
    }

    case 2:{


        if (head){ cout<<"moving head 0.4"<<endl;

            headMove = node->advertise<std_msgs::Float64MultiArray>(headTopic, 1);
            sleep(1);

            std_msgs::Float64MultiArray newPos;
            if(ros::ok) {
                float newP[7] = {0.4, 0, 0, 0, 0, 0, 0};
                for(int i = 0; i < 7; ++i){newPos.data.push_back(newP[i]); }

                headMove.publish(newPos);
                ros::spinOnce();

                cout<<"head moved"<<endl;
            }
        }

        //arTag tracker

        ros::Subscriber markerSub = node->subscribe("/tf", 1, arCallback);
        ros::Rate lRate(1);

        cout<<"subscriber for ar tags created"<<endl;

        while(!firstSet){
            ros::spinOnce();
            lRate.sleep();}

        queue->stopCurrentMode();
        raThr = queue->startQueueThread();
        queue->setStiffness(0.2, 0.01, 0.2, 15000, 150, pickMaxAxisTorque);

        queue->switchMode(30);

        std::thread* inputThr = NULL;
        inputThr = new std::thread(consoleInputter);

        cout<<"put the robot in start position "<<endl;
        cout<<"press any key for start of measurement: "<<endl;

        while(consoleInput == 0) { usleep(0.5 * 1e4); }


        sleep(5);

        std::thread* inputThrC = NULL;
        inputThrC = new std::thread(collectDataReal,queue, fileR, fileO, fileS);

        queue->stopCurrentMode();
        queue->switchMode(10);

        vector<double> startP={0.272241000000000,	0.876041000000000,	0.395563000000000,	0.776795000000000,	0.0112587000000000,	-0.0194455000000000,	0.629462000000000};


        cout << "moving to the start position "<< endl;
        queue->moveCartesian(vectordouble2pose(&startP));
        t_executor_res demoRes= executePush(queue, fileDMP, az, bz);
        cout << "movement done "<< endl;

        break;
    }

    case 3: {
        std::shared_ptr<SimInterface> simI= std::shared_ptr<SimInterface>(new SimInterface (argc, args, kukaStepWaitTime, *node));
        cout<<"simulator interface created"<<endl;

        float newP[3]={0.5,0.7,0.01};
        float newO[4]={0,0,0,0};
        float dim[3]= {1,2,0.08};

        simI->addPrimShape(1,"sponge", newP, newO, dim, 10.0);
        simI->setObjMaterial("sponge","highFrictionMaterial");
        cout<< "sponge imported "<< endl;

        queue->stopCurrentMode();
        queue->switchMode(10);

        //vector<double> startP={0.272241000000000,	0.876041000000000,	0.395563000000000,	0.776795000000000,	0.0112587000000000,	-0.0194455000000000,	0.629462000000000};
        //vector<double> startP={0.10626,	1.08934,	0.305562,	-0.216973,	-0.9353,	0.206316,	0.188599};

        vector<double> startP={0.0605175,	0.744077,	0.288841,	-0.245374,	-0.954758,	0.141078,	0.0912608};
        vector<double> newPoC1={0.272241000000000,	0.876041000000000,	0.395563000000000,	0.776795000000000,	0.0112587000000000,	-0.0194455000000000,	0.629462000000000};

        cout << "moving to the start position "<< endl;
        // queue->moveCartesian(vectordouble2pose(&startP));
        queue->moveCartesian(vectordouble2pose(&newPoC1));

        float newP1[3]={0.30,0.7,0.2};
        //float newP1[3]={0.23,0.7,0.2};
        //float newP1[3]={0.25,0.7,0.2};
        float newO1[4]={0,0,0,0};
        float dim1[3]= {0.2,0.2,0.1};

        string objectId="box";
        simI->addPrimShape(1,objectId,newP1,newO1, dim1,1);

        cout <<" box imported " << endl;

        start = 1;
        std::thread* inputThrC = NULL;
        inputThrC = new std::thread(collectDataSim, queue, simI,  objectId, fileR,  fileO, fileS);

        t_executor_res demoRes= executePush(queue, fileDMP, az, bz);

        cout << "movement done "<< endl;

        break;

    }


        /*



        while(true) {

            ros::spinOnce();
            pfile<<translation.x;
            pfile<< "\t" << translation.y;
            pfile<< "\t" << translation.z;
            pfile<< "\t" << rotation.x;
            pfile<< "\t" << rotation.y;
            pfile<< "\t" << rotation.z;
            pfile<< "\t" << rotation.w;
            pfile<<endl;
            lRate.sleep();

        }


        if (head){ cout<<"moving head 0.4"<<endl;
            // cout<<"HELOOOOOO";

            headMove = node.advertise<std_msgs::Float64MultiArray>(headTopic, 1);

            sleep(1);

            std_msgs::Float64MultiArray newPos;
            if(ros::ok) {
                float newP[7] = {0.4, 0, 0, 0, 0, 0, 0};
                for(int i = 0; i < 7; ++i){newPos.data.push_back(newP[i]); }

                headMove.publish(newPos);
                ros::spinOnce();

                ros::spinOnce();

                cout<<"head moved"<<endl;
            }
        }
        //teaching collecting data
        HandSchunk* handR=new HandSchunk(argc, args, kukaStepWaitTime, env, arm+"_sdh", node);
        float newPos[7] = {0, -1.57, 0, -1.57,0,-1.57,0};

        handR->moveFingers(newPos);
        // mes_result r1=handR->getHandData();
        // handR->grip(2,0.05);
        OrocosControlQueueExt* queue = new OrocosControlQueueExt(argc, args, kukaStepWaitTime, env, arm+"_arm", node);
        queue->stopCurrentMode();

        cout<<"current mode stop"<<endl;
        queue->startQueueThread();
        queue->switchMode(10);

        //float startPos[7] = {0.141948,	1.13384,	0.995718,	1.41301,	1.38431,	-0.972765, 1.776939};
        float startPos[7]={-0.8401662707328796, 1.1858383417129517, 0.015342351980507374, 0.615711510181427, 2.8110413551330566, -2.039827346801758, -0.3832871913909912};
        queue->moveJoints(startPos);


        queue->stopCurrentMode();

        queue->setStiffness(0.2, 0.01, 0.2, 15000, 150, pickMaxAxisTorque);

        queue->switchMode(30);

        cout<<"mode switched to 30"<<endl;

        ofstream oFile;
        ofstream sFile;
        oFile.open("/home/c7031098/testing/data071114.txt");
        sFile.open("/home/c7031098/testing/Rcart071114.txt");

        double time = 0.0;
        double time1 = 0.0;
        double lastTime = -1.0;
        double lastTime1 = -1.0;
        float* joints;
        float* wrench;
        float* w;
        int columns = 8;

        mes_result mesRes = queue->getCurrentJoints();
        mes_result mesRes1 = queue->getSensorWristData();
        mes_result mesRes2= queue->getCurrentQuatPose();
        time = mesRes.time;
        joints = mesRes.joints;

        time1 = mesRes1.time;
        wrench = mesRes1.joints;

        w=mesRes2.joints;


        usleep(0.5 * 1e4);
        if(joints != NULL && lastTime != time) {

            oFile << time;
            for(int i = 0; i < columns - 1; ++i) { oFile << "\t" << joints[i]; } //joint
            oFile << endl;
            lastTime = time;

            sFile << time1;
            for(int i = 0; i < columns - 1; ++i) { sFile << "\t" << w[i]; } //position cartesian
            sFile << endl;
            lastTime1 = time1;

        }



        queue->stopCurrentMode();
        queue->switchMode(10);
        queue->stopCurrentMode();
        break;
    }*/
    }

    getch();

    return 0;

}


void arCallback(tf::tfMessage msg) {

    firstSet = true;
    newTarget = msg;
    geometry_msgs::TransformStamped t = msg.transforms.at(0);
    translation = t.transform.translation;
    rotation = t.transform.rotation;

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
    vector<geometry_msgs::Pose> robotP;
    vector<geometry_msgs::Pose> objectP;
    vector <double> tvec;

    while (start && !record){
        if (record) start = false;

    }

    while(record){


        //mes_result mesResJ = queue->getCurrentJoints();
       // geometry_msgs::Pose PoseR = queue->getCartesianPose();
       // geometry_msgs::Pose Pose = sim->getObjPose(objectId);
        //mes_result mesResS = queue->getCurrentCartesianFrcTrq();

        time = ros::Time::now().toSec() - start;
        tvec.push_back(time);
        robotP.push_back(queue->getCartesianPose());
        objectP.push_back(sim->getObjPose(objectId));

         usleep(0.5 * 1e3);

    }


   cout << "writing to files recorded data; data length " << tvec.size() << endl;

   for (int i=0; i<tvec.size(); i++){
    rFile << tvec.at(i);
    rFile << "\t" << robotP.at(i).position.x;
    rFile << "\t" << robotP.at(i).position.y;
    rFile << "\t" << robotP.at(i).position.z;
    rFile << "\t" << robotP.at(i).orientation.x;
    rFile << "\t" << robotP.at(i).orientation.y;
    rFile << "\t" << robotP.at(i).orientation.z;
    rFile << "\t" << robotP.at(i).orientation.w;
    rFile << endl;


    oFile << tvec.at(i);
    oFile << "\t" << objectP.at(i).position.x;
    oFile << "\t" << objectP.at(i).position.y;
    oFile << "\t" << objectP.at(i).position.z;
    oFile << "\t" << objectP.at(i).orientation.x;
    oFile << "\t" << objectP.at(i).orientation.y;
    oFile << "\t" << objectP.at(i).orientation.z;
    oFile << "\t" << objectP.at(i).orientation.w;
    oFile << endl;

    }

   cout << " writing done " << endl;




}

void collectDataReal(std::shared_ptr<OrocosControlQueue> queue, string fileR, string fileO, string fileS){
    ofstream oFile;
    ofstream sFile;
    ofstream rFile;

    oFile.open(fileO);
    sFile.open(fileS);
    rFile.open(fileR);

    double time = 0.0;
    double lastTime = -1.0;

    arma::vec wrench;
    int columns = 8;



    while(1){



        // mes_result mesRes = queue->getCurrentJoints();
        geometry_msgs::Pose PoseR = queue->getCartesianPose();
        mes_result mesResS = queue->getCurrentCartesianFrcTrq();
        mes_result joints = queue->getCurrentJoints();

        time = joints.time;
        wrench = mesResS.joints;


        usleep(0.5 * 1e4);
        if(lastTime != time) {

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
            oFile << "\t" << translation.x;
            oFile << "\t" << translation.y;
            oFile << "\t" << translation.z;
            oFile << "\t" << rotation.x;
            oFile << "\t" << rotation.y;
            oFile << "\t" << rotation.z;
            oFile << "\t" << rotation.w;
            oFile << endl;

            lastTime = time;


        }
    }



}

char consoleInputter() {
    cin >> consoleInput;
    return consoleInput;
}

t_executor_res executePush(shared_ptr<OrocosControlQueue> movementQu, string file, double az, double bz) {

    Gnuplot* g1 = NULL;

    vector<double> tmpmys{0, 1, 2, 3, 4, 4.1, 4.3, 4.5};
    //vector<double> tmpmys;
    vector<double> tmpsigmas{0.2, 0.8};

    // reading in file
    mat carts = readMovements(file);

    for (int i=1; i <carts.n_rows; i++){


    }
    tmpmys = constructDmpMys(carts);

    double tau = 0.8;
    double ax = -log((float)0.1) / carts(carts.n_rows - 1, 0) / tau;

    vector<DMPBase> baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);

    TrajectoryDMPLearner dmpLearner(baseDef, tau, az, bz, ax, carts);
    Dmp learnedDmps = dmpLearner.fitTrajectories();
    arma::vec startP=learnedDmps.getY0();

    movementQu->moveCartesian(vectorarma2pose(&startP));
    movementQu->stopCurrentMode();
    movementQu->switchMode(20);

    DMPExecutor dmpexec(learnedDmps, movementQu, 0);
    record = true;
    t_executor_res dmpResult = dmpexec.executeTrajectory(2);

    record = false;


    return dmpResult;

}



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


/*void collectDataSim(std::shared_ptr<OrocosControlQueue> queue,std::shared_ptr<SimInterface> sim, string objectId, string fileR, string fileO, string fileS){

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
    vector<geometry_msgs::Pose> robotP;
    vector<geometry_msgs::Pose> objectP;

    while(record){



        mes_result mesResJ = queue->getCurrentJoints();
       // geometry_msgs::Pose PoseR = queue->getCartesianPose();
       // geometry_msgs::Pose Pose = sim->getObjPose(objectId);
        //mes_result mesResS = queue->getCurrentCartesianFrcTrq();

        time = ros::Time::now().toSec() - start;
        wrench = mesResS.joints;


       // usleep(0.5 * 1e4);
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




}*/
