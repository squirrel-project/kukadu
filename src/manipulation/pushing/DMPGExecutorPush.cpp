#include "DMPGExecutorPush.h"

using namespace std;
using namespace arma;

DMPExecutorPush::DMPExecutorPush( KUKADU_SHARED_PTR<Dmp> dmpL, KUKADU_SHARED_PTR<ControlQueue> execQueue, string env, KUKADU_SHARED_PTR<SimInterface> simI, string objectID): DMPExecutor(dmpL, execQueue), objectID(objectID), simI(simI) {

    this->constructPush(env);

}

DMPExecutorPush::DMPExecutorPush( KUKADU_SHARED_PTR<Dmp> dmp, KUKADU_SHARED_PTR<ControlQueue> execQueue, string env, KUKADU_SHARED_PTR<VisionInterface> visI): DMPExecutor(dmp, execQueue), visI(visI) {

    this->constructPush(env);

}

void DMPExecutorPush::constructPush(string env) {
    if (env == "real") doSimulation = false;
    else doSimulation = true;
    timeCount = 0;
    setT0 = false;
    startMov = false;

    Ts = 0.5;
    stopObj = false;
    thr = KUKADU_SHARED_PTR<kukadu_thread>();
    thr.reset();

    pcF = zeros(50);
    pcDist = zeros(50);
    pcFdist = zeros(50,50);

}

double DMPExecutorPush::addTerm(double t, const double* currentDesiredYs, int jointNumber, KUKADU_SHARED_PTR<ControlQueue> queue) {

    contPosition = true;
    contOrient = true;
    contForce = true;
    orientTerm = true;
    startMov = true;

    double corr = 0;
    double corrP = 0;
    double corrO = 0;
    double corrF = 0;

    double KxP = 0.1;
    double KyP = 0.1;
    double KxO = 0.02;
    double KxF = 0.0001;
    double Kalfa = 0.1;

    if(contPosition && (eXo.size() > 1))
        KxP = 0.005 * EeXo / vareXo;

    if(contForce && (Fcart.size() > 1)){

        //KxF = gradFx * varFcart;
        double pr;
        if (gradFx[gradFx.size() - 1] > 0.0) pr = 1.0;

        KxF = pr * 0.0001;

    }



    if(jointNumber == 1) {

        if(contPosition && (eXo.size() > 1)) corrP =   KxP * eXo[eXo.size() - 1];
        if(contForce && (Fcart.size() > 1)) corrF = KxF * (EFdmin - Fcart[Fcart.size() - 1]);
        if(contOrient && (eXo.size() > 1)) corrO = KxO * (EthO - tf::getYaw(currentObjPose.orientation)) / vareThO;

    }

    corr = corrP + corrO + corrF;

    corr = corr * 100;
    double tmp = 1.0;
    if((jointNumber) && (corr > tmp)) corr = tmp;
    if((jointNumber) && (corr < -1*tmp)) corr = -1*tmp;

    if(jointNumber == 1) return corr;
    else return 0.0;

}

void DMPExecutorPush::setObjectData(arma::vec times, arma::mat cartPos){

    ros::spinOnce();
    sleep(0.5);

    newF = controlQueue->getAbsoluteCartForce();
    oldF = newF;
    currentRobPose =  controlQueue->getCartesianPose();
    oldRobPose = currentRobPose;

    if (doSimulation)
        currentObjPose = simI->getObjPose(objectID);

    else currentObjPose = visI->getArPose();

    this->ObjTimes = times - times(0);
    this->ObjCartPos = cartPos;
    Ts = ObjTimes(1) - ObjTimes(0);
    Lx1 = currentObjPose.position.x;
    Ly1 = currentObjPose.position.y;
    Lx2 = cartPos(cartPos.n_rows - 1, 0) + currentObjPose.position.x - cartPos(0, 0);
    Ly2 = cartPos(cartPos.n_rows - 1, 1) + currentObjPose.position.y - cartPos(0, 1);;
    Th0 = tf::getYaw(currentObjPose.orientation);

    thr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&DMPExecutorPush::updateData, this));


}
void DMPExecutorPush::stopObject() {
    stopObj = true;
}

void DMPExecutorPush::updateData() {
    double t = 0.0;
    while(!stopObj){

        if(startMov){

            time.push_back(t);

            if (doSimulation) {
                currentObjPose = simI->getObjPose(objectID);
            }
            else currentObjPose = visI->getArPose();
            currentRobPose = controlQueue->getCartesianPose();

            xR.push_back(currentRobPose.position.x);
            yR.push_back(currentRobPose.position.y);

            xO.push_back(currentObjPose.position.x);
            yO.push_back(currentObjPose.position.y);

            distO.push_back(Lx1 - currentObjPose.position.x);
            int idmin = 49;
            for (int i = 0; i < distO.size(); ++i){
                if (distO[i] < idmin / 100){
                    idmin = int(distO[i] * 100);
                }
            }
            if (idmin < 0) idmin = 0;


            newF = controlQueue->getAbsoluteCartForce();
            if (doSimulation)newF = newF / 10;
            Fcart.push_back(newF);
            double grad = 0;
            if ((currentRobPose.position.x - oldRobPose.position.x)!= 0) grad = (newF - oldF) / (currentRobPose.position.x - oldRobPose.position.x);
            gradFx.push_back(grad);
            if (currentRobPose.position.x != oldRobPose.position.x) oldRobPose = currentRobPose;

            EFcart = std::accumulate(Fcart.begin(), Fcart.end(), 0.0) / Fcart.size();
            vEFcart.push_back(EFcart);
            if(Fcart.size() > 2){
                int d, f;
                if (distO[distO.size() - 1] <= 0.1) d = 0;
                else if (distO[distO.size() - 1] <= 0.49)  d = int(distO[distO.size() - 1] * 100);
                else d = 49;

                if (Fcart[Fcart.size() - 2] < 1.0) f = 0;
                else if (Fcart[Fcart.size() - 2] <= 49.0) f = int(Fcart[Fcart.size() - 2]);
                else f = 49;

                pcDist(d) = pcDist(d) + 1;
                pcF(f) = pcF(f) + 1;
                pcFdist(f, d) = pcFdist(f, d) + 1;

                vec scale = pcFdist.col(idmin) / pcDist(idmin);
                EFdmin = 0;
                for (int i = 0; i < 50; ++i) EFdmin = EFdmin + scale(i) * i / 100;
            }

            thO.push_back(tf::getYaw(currentObjPose.orientation));
            EthO = std::accumulate(thO.begin(), thO.end(), 0.0) / thO.size();
            eThO.push_back(EthO - tf::getYaw(currentObjPose.orientation));
            EeThO = std::accumulate(eThO.begin(), eThO.end(), 0.0) / eThO.size();

            // arma::vec p = pointOnLine2Point(currentObjPose.position.x, currentObjPose.position.y, Lx1, Ly1, Lx2, Ly2);
            //cout << p<<endl;
            //eXo.push_back(p(0) - currentObjPose.position.x);
            eXo.push_back(Lx1 - currentObjPose.position.x);
            // eYo.push_back(p(1) - currentObjPose.position.y);
            EeXo = std::accumulate(eXo.begin(), eXo.end(), 0.0) / eXo.size();
            //EeYo = std::accumulate(eYo.begin(), eYo.end(), 0.0) / eYo.size();


            vector<double> diffX, diffY, diffThO, diffF;
            diffX.resize(eXo.size());
            diffThO.resize(eThO.size());

            std::transform(eXo.begin(), eXo.end(), diffX.begin(), bind2nd(std::plus<double>(), - EeXo));
            vareXo = std::inner_product(diffX.begin(), diffX.end(), diffX.begin(), 0.0) / (diffX.size() - 1);


            std::transform(eThO.begin(), eThO.end(), diffThO.begin(), bind2nd(std::plus<double>(), - EeThO));
            vareThO = std::inner_product(diffThO.begin(), diffThO.end(), diffThO.begin(), 0.0) / (diffThO.size() - 1);

        }

        sleep(10 * Ts);
        t = t + 10 * Ts;
    }



}

int DMPExecutorPush::findIndex(double t, vec times) {

    int ind = 0;
    for (int i = 0; i < times.n_elem; ++i){
        if (t >= times(i)) ind = i;
    }
    return ind;

}

void  DMPExecutorPush::saveData(string path) {

    std::ofstream pStream , pStreamP;
    pStream.open((path + string("/") + "pushRes.txt").c_str());
    pStreamP.open((path + string("/") + "pushResP.txt").c_str());

    pStreamP << "xR" << "\t";
    for (int i = 0; i < xR.size(); ++i) pStreamP << xR[i] << "\t";
    pStreamP << endl;

    pStreamP << "yR" << "\t";
    for (int i = 0; i < yR.size(); ++i) pStreamP << yR[i] << "\t";
    pStreamP << endl;

    pStreamP << "xO" << "\t";
    for (int i = 0; i < xO.size(); ++i) pStreamP << xO[i] << "\t";
    pStreamP << endl;

    pStreamP << "yO" << "\t";
    for (int i = 0; i < yO.size(); ++i) pStreamP << yO[i] << "\t";
    pStreamP << endl;

    pStream << "distO" << "\t";
    for (int i = 0; i < distO.size(); ++i) pStream << distO[i] << "\t";
    pStream << endl;

    pStream << "eXo" << "\t";
    for (int i = 0; i < eXo.size(); ++i) pStream << eXo[i] << "\t";
    pStream << endl;

    pStream << "thO" << "\t";
    for (int i = 0; i < thO.size(); ++i) pStream << thO[i] << "\t";
    pStream << endl;

    pStream << "eThO" << "\t";
    for (int i = 0; i < eThO.size(); ++i) pStream << eThO[i] << "\t";
    pStream << endl;

    pStream << "Fcart" << "\t";
    for (int i = 0; i < Fcart.size(); ++i) pStream << Fcart[i] << "\t";
    pStream << endl;

    pStream << "vFcart" << "\t";
    for (int i = 0; i < vEFcart.size(); ++i) pStream << vEFcart[i] << "\t";
    pStream << endl;

    pStream << "pcF" << "\t";
    for (int i = 0; i < 50; ++i) pStream << pcF(i) << "\t";
    pStream << endl;

    pStream << "pcDist" << "\t";
    for (int i = 0; i < 50; ++i) pStream << pcF(i) << "\t";
    pStream << endl;

    pStream << "pcF" << "\t";
    for (int i = 0; i < 50; ++i) pStream << pcF(i) << "\t";
    pStream << endl;

    pStream.close();
    pStreamP.close();

}


