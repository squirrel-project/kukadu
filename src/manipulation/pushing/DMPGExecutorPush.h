#ifndef DMPGEXECUTORPUSH_H
#define DMPGEXECUTORPUSH_H

#include "../../../kukadu/include/kukadu.h"
#include "../../../kukadu/src/types/DMP.h"
#include "../../../kukadu/src/utils/utils.h"

#include <cstdio>
#include <iostream>
#include <fstream>
#include <thread>
#include <string>
#include <vector>
#include <wordexp.h>
#include "ros/ros.h"
#include "std_msgs/Int32.h"


using namespace std;
using namespace arma;

class DMPExecutorPush : public DMPExecutor {

private:

    string objectID;
    std::shared_ptr<SimInterface>  simI;
    std::shared_ptr<VisionInterface> visI;
    //std::shared_ptr<ControlQueue> queue;
    std::shared_ptr<std::thread> thr;

    bool doSimulation;
    bool stopObj;
    bool setT0;
    bool startMov;

    bool contPosition;
    bool contOrient;
    bool contForce;
    bool orientTerm;

    int timeCount;
    double Ts;
    double T0;
    double Th0;

    bool setOrientCorr;
    bool setPosCorr;

    string environment;


    mat DesPos;
    vec ObjTimes;
    mat ObjCartPos;

    geometry_msgs::Pose currentObjPose;
    geometry_msgs::Pose currentRobPose;
    geometry_msgs::Pose oldRobPose;


    vector<double> thO;
    vector<double> xO;
    vector<double> yO;
    vector<double> thR;
    vector<double> xR;
    vector<double> yR;
    vector<double> eThO;
    vector<double> eXo;
    vector<double> eYo;
    vector<double> Fcart;
    vector<double> vEFcart;
    vector<double> distO;
    vector<double> time;


    double EthO;
    double EeThO;
    double EeXo;
    double EeYo;
    double EFcart;
    double EFdmin;
    double vareXo;
    double vareYo;
    double vareThO;
    double varFcart;
    vector<double> gradFx;
    double oldF, newF;

    arma::vec pcF;
    arma::vec pcDist;
    arma::mat pcFdist;


    double Lx1, Ly1, Lx2, Ly2;

    void updateData();
    int findIndex(double t, vec times);
    void constructPush(string env);




public:

    DMPExecutorPush( std::shared_ptr<Dmp> dmp, std::shared_ptr<ControlQueue> execQueue, string env, std::shared_ptr<SimInterface> simI,string objectID);
    DMPExecutorPush( std::shared_ptr<Dmp> dmp, std::shared_ptr<ControlQueue> execQueue, string env, std::shared_ptr<VisionInterface> visI);
    double addTerm(double t, const double* currentDesiredYs, int jointNumber, std::shared_ptr<ControlQueue> queue);
    void setObjectData(arma::vec times, arma::mat cartPos);
    void saveData(string path);
    void stopObject();

};



#endif // DMPGEXECUTORPUSH_H
