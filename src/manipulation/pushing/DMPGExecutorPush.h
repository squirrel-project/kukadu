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

class DMPExecutorPush : public DMPExecutor {

private:

    std::string objectID;
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

    std::string environment;


    arma::vec ObjTimes;
    arma::mat ObjCartPos;
    arma::mat DesPos;

    geometry_msgs::Pose currentObjPose;
    geometry_msgs::Pose currentRobPose;
    geometry_msgs::Pose oldRobPose;


    std::vector<double> thO;
    std::vector<double> xO;
    std::vector<double> yO;
    std::vector<double> thR;
    std::vector<double> xR;
    std::vector<double> yR;
    std::vector<double> eThO;
    std::vector<double> eXo;
    std::vector<double> eYo;
    std::vector<double> Fcart;
    std::vector<double> vEFcart;
    std::vector<double> distO;
    std::vector<double> time;
    std::vector<double> gradFx;


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
    double oldF, newF;
    double Lx1, Ly1, Lx2, Ly2;

    arma::vec pcF;
    arma::vec pcDist;
    arma::mat pcFdist;

    void updateData();
    void constructPush(std::string env);

    int findIndex(double t, arma::vec times);




public:

    DMPExecutorPush(std::shared_ptr<Dmp> dmp, std::shared_ptr<ControlQueue> execQueue, std::string env, std::shared_ptr<SimInterface> simI, std::string objectID);
    DMPExecutorPush(std::shared_ptr<Dmp> dmp, std::shared_ptr<ControlQueue> execQueue, std::string env, std::shared_ptr<VisionInterface> visI);

    void stopObject();
    void saveData(std::string path);
    void setObjectData(arma::vec times, arma::mat cartPos);

    double addTerm(double t, const double* currentDesiredYs, int jointNumber, std::shared_ptr<ControlQueue> queue);

};



#endif // DMPGEXECUTORPUSH_H
