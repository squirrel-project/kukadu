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

    if(!setT0) {
        T0 = t;
        setT0 = true;
    }

    double corr = 0;
    double Kx = 0.2;


    if (jointNumber == 1) {

        if (ObjTimes(ObjTimes.n_elem -1) > t - T0 -Ts)
            corr = Kx * (ObjCartPos((int) (t - T0) / Ts, 0) - currentObjPose.position.x);
        cout <<" (int) (t - T0) / Ts "<< (int) (t - T0) / Ts<<endl;
        cout<<"ObjTimes.n_elem "<<ObjTimes.n_elem<<endl;
        cout<<"ObjTimes(ObjTimes.n_elem -1) "<<ObjTimes(ObjTimes.n_elem -1) <<endl;
        cout<<t - T0 -Ts<<endl;



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
