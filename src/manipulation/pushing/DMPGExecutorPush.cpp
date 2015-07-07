#include "DMPGExecutorPush.h"

DMPExecutorPush::DMPExecutorPush( std::shared_ptr<Dmp> dmpL, std::shared_ptr<ControlQueue> execQueue, string env, std::shared_ptr<SimInterface> simI, string objectID): DMPExecutor(dmpL, execQueue), objectID(objectID), simI(simI)
{
    if (env == "real") doSimulation = false;
    else doSimulation = true;
    timeCount = 0;
    setT0 = false;

    if (doSimulation){
        Ts = 0.5;
        stopObj = false;
        thr = shared_ptr<thread>(nullptr);
        thr = std::shared_ptr<std::thread>(new std::thread(&DMPExecutorPush::getPoseSim, this));
    }



}
double DMPExecutorPush::addTerm(double t, const double* currentDesiredYs, int jointNumber, std::shared_ptr<ControlQueue> queue){

    double corr = 0;

    double Kx = 0.5;


    if (jointNumber == 1) {

        if (findIndex(t, ObjTimes))
            corr = - Kx * (ObjCartPos(findIndex(t, ObjTimes), 0) - currentObjPose.position.x);

    }


    return corr;
}
void DMPExecutorPush::setObjectData(arma::vec times, arma::mat cartPos){
    this->ObjTimes = times;
    this->ObjCartPos = cartPos;
     Ts = ObjTimes(1)-ObjTimes(0);



}

void DMPExecutorPush::getPoseSim(){
    while(!stopObj){
        currentObjPose = simI->getObjPose(objectID);
        sleep(0.5 * Ts);
    }


}

int DMPExecutorPush::findIndex(double t, vec times){

    int ind = 0;
    for (int i = 0; i < times.n_elem; ++i){
        if (t >= times(i)) ind = i;
    }
    return ind;
}
