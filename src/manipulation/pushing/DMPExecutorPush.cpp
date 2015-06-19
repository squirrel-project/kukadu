#include "DMPExecutorPush.h"

DMPExecutorPush::DMPExecutorPush(Dmp dmp, std::shared_ptr<ControlQueue> execQueue, int doSimulation, std::shared_ptr<SimInterface> simI,string object_id): DMPExecutor(dmp, execQueue), object_id(object_id), doSimulation(doSimulation), simI(simI)
{}
double DMPExecutorPush::addTerm(double t, const double* currentDesiredYs, int jointNumber, std::shared_ptr<ControlQueue> queue){

        return 0;
    }
