#ifndef DMPEXECUTORPUSH_H
#define DMPEXECUTORPUSH_H

#include "../../../kukadu/include/kukadu.h"

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

    string object_id;
    std::shared_ptr<SimInterface>  simI;
    int doSimulation;
    mat DesPos;

public:

    DMPExecutorPush(Dmp dmp, std::shared_ptr<ControlQueue> execQueue, int doSimulation, std::shared_ptr<SimInterface> simI,string object_id);

    double addTerm(double t, const double* currentDesiredYs, int jointNumber, std::shared_ptr<ControlQueue> queue) override;

};

#endif // DMPEXECUTORPUSH_H
