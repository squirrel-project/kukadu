#include "DMPExecutorPush.h"

DMPExecutorPush::DMPExecutorPush(vector<Dmp> dmpLib, std::shared_ptr<ControlQueue> execQueue, int doSimulation, std::shared_ptr<SimInterface> simI,string object_id): DMPExecutor(dmpLib.at(0), execQueue),DmpLib(dmpLib), object_id(object_id), doSimulation(doSimulation), simI(simI)
{

    for (int j=0; j < DmpLib.size(); j++){

        std::vector<arma::vec> coeffs = dmpLib.at(j).getDmpCoeffs();
        this->dmpLibCoeffs.push_back(coeffs);

        cout <<" executor created " << endl;
        //cout <<" dmpLibC vec size" << dmpLib.size() << endl;
        //cout <<" dmpLibCoeffs vec size" << dmpLibCoeffs.size() << endl;
    }
}
double DMPExecutorPush::addTerm(double t, const double* currentDesiredYs, int jointNumber, std::shared_ptr<ControlQueue> queue){

    return 0;
}

int DMPExecutorPush::func(double t, const double* y, double* f, void* params) {


    // y' = z / tau
    // z' = 1 / tau * ( az * (bz * (g - y) - z) + f);
    // x' = -ax / tau * x

  //    cout <<" func called " <<  endl;

    for(int i = 0; i < odeSystemSizeMinOne; i = i + 2) {

        double yPlusOne = y[i + 1];

        int currentSystem = (int) (i / 2);
        f[i] = yPlusOne * oneDivTau;
        double g = gs(currentSystem);

        std::vector<arma::vec> currentCoeffs;

        for (int j=0; j < DmpLib.size(); j++) currentCoeffs.push_back(dmpLibCoeffs.at(j).at(currentSystem));

        vector<double>  addTerm ;

       // cout << " DmpLib.at(0).getTmax() " << DmpLib.at(0).getTmax() <<endl;

        if(t <= (DmpLib.at(0).getTmax() - 1)) {


            for (int j = 0; j < DmpLib.size(); j++) addTerm.push_back(trajGen->evaluateByCoefficientsSingleNonExponential(y[odeSystemSizeMinOne], currentCoeffs.at(j)));
            vector<double> weights;
            for (int j = 0; j < DmpLib.size(); j++) weights.push_back(1.0);

            double addTermSum = 0;

            for (int j = 0; j<DmpLib.size(); j++) addTermSum = addTermSum + weights.at(j)*addTerm.at(j);
            addTermSum = addTermSum / std::accumulate(weights.begin(), weights.end(),0.0);

            f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne) + addTermSum)  + this->addTerm(t, y, i / 2, controlQueue);

        } else {
            //	cout << "(DMPExecutor) executing dmp over teaching duration" << endl;
            //    throw "stopped dmp execution";
            f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne));
        }

    }

    if(this->simulate == EXECUTE_ROBOT) {

        currentJoints = controlQueue->getCurrentJoints().joints;
        double corrector = 0.0;

        if(!usesExternalError()) {

            // include original  phase stopping
            for(int i = 0; i < degofFreedom; ++i) previousDesiredJoints[i] =  y[2 * i];
            double dist = computeDistance(previousDesiredJoints, currentJoints);

            corrector = 1.0 + ac * dist;

        } else {
            corrector = 1.0 + ac * getExternalError();
        }

        f[odeSystemSizeMinOne] = - axDivTau * y[odeSystemSizeMinOne] / corrector;

    }

    else if(this->simulate == SIMULATE_DMP) {

        // progress as usual
        f[odeSystemSizeMinOne] = - axDivTau * y[odeSystemSizeMinOne];

    }

    return GSL_SUCCESS;

}
