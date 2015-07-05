#include "DMPGExecutorPush.h"

DMPExecutorPush::DMPExecutorPush( std::shared_ptr<Dmp> dmpL, std::shared_ptr<ControlQueue> execQueue, string env, std::shared_ptr<SimInterface> simI, string objectID): DMPExecutor(dmpL, execQueue), objectID(objectID), simI(simI)
{
    if (env == "real") doSimulation = false;
    else doSimulation = true;

        cout <<" executor created " << endl;


}
double DMPExecutorPush::addTerm(double t, const double* currentDesiredYs, int jointNumber, std::shared_ptr<ControlQueue> queue){


    return 0;
}
void DMPExecutorPush::setObjectData(arma::vec times, arma::mat cartPos){
    this->ObjTimes = times;
    this->ObjCartPos = cartPos;

}
