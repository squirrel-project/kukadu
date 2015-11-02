#ifndef KUKADU_DMPGEXECUTORPUSH_H
#define KUKADU_DMPGEXECUTORPUSH_H

#include "../../types/DMP.h"
#include "../../utils/utils.h"
#include "../../types/KukaduTypes.h"
#include "../../robot/SimInterface.h"
#include "../../robot/VisionInterface.h"
#include "../../trajectory/DMPExecutor.h"

#include <cstdio>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <wordexp.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

class DMPExecutorPush : public DMPExecutor {

private:

    std::string objectID;

    KUKADU_SHARED_PTR<kukadu_thread> thr;
    KUKADU_SHARED_PTR<SimInterface>  simI;
    KUKADU_SHARED_PTR<VisionInterface> visI;



    bool setT0;
    bool stopObj;
    bool startMov;
    bool contForce;
    bool contOrient;
    bool orientTerm;
    bool setPosCorr;
    bool contPosition;
    bool doSimulation;
    bool setOrientCorr;

    int timeCount;

    double Ts;
    double T0;
    double Th0;
    double EeXo;
    double EeYo;
    double EthO;
    double EeThO;
    double EFcart;
    double EFdmin;
    double vareXo;
    double vareYo;
    double vareThO;
    double varFcart;
    double oldF, newF;
    double Lx1, Ly1, Lx2, Ly2;

    std::string environment;

    arma::vec ObjTimes;
    arma::mat ObjCartPos;
    arma::mat DesPos;

    geometry_msgs::Pose oldRobPose;
    geometry_msgs::Pose currentObjPose;
    geometry_msgs::Pose currentRobPose;

    std::vector<double> xO;
    std::vector<double> yO;
    std::vector<double> xR;
    std::vector<double> yR;
    std::vector<double> thO;
    std::vector<double> thR;
    std::vector<double> eXo;
    std::vector<double> eYo;
    std::vector<double> eThO;
    std::vector<double> time;
    std::vector<double> Fcart;
    std::vector<double> distO;
    std::vector<double> gradFx;
    std::vector<double> vEFcart;

    arma::vec pcF;
    arma::vec pcDist;
    arma::mat pcFdist;

    void updateData();
    void constructPush(std::string env);

    int findIndex(double t, arma::vec times);

public:

    DMPExecutorPush(KUKADU_SHARED_PTR<Dmp> dmp, KUKADU_SHARED_PTR<ControlQueue> execQueue, std::string env, KUKADU_SHARED_PTR<SimInterface> simI, std::string objectID);
    DMPExecutorPush(KUKADU_SHARED_PTR<Dmp> dmp, KUKADU_SHARED_PTR<ControlQueue> execQueue, std::string env, KUKADU_SHARED_PTR<VisionInterface> visI);

    void stopObject();
    void saveData(std::string path);
    void setObjectData(arma::vec times, arma::mat cartPos);

    double addTerm(double t, const double* currentDesiredYs, int jointNumber, KUKADU_SHARED_PTR<ControlQueue> queue);

};



#endif
