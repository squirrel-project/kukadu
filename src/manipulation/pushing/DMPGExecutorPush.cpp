#include "DMPGExecutorPush.h"

DMPExecutorPush::DMPExecutorPush( std::shared_ptr<Dmp> dmpL, std::shared_ptr<ControlQueue> execQueue, int doSimulation, std::shared_ptr<SimInterface> simI, string object_id): DMPExecutor(dmpL, execQueue), object_id(object_id), doSimulation(doSimulation), simI(simI)
{


        cout <<" executor created " << endl;


}
double DMPExecutorPush::addTerm(double t, const double* currentDesiredYs, int jointNumber, std::shared_ptr<ControlQueue> queue){

    return 0;
}
