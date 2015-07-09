#include "DMPExecutor.h"
#include "tf/transform_datatypes.h"

using namespace std;
using namespace arma;

DMPExecutor::DMPExecutor(std::shared_ptr<Dmp> traj, std::shared_ptr<ControlQueue> execQueue) {

    construct(traj, execQueue, 1);

}

DMPExecutor::DMPExecutor(std::shared_ptr<Dmp> traj, std::shared_ptr<ControlQueue> execQueue, int suppressMessages) {

    construct(traj, execQueue, suppressMessages);

}

DMPExecutor::DMPExecutor(std::shared_ptr<Trajectory> traj, std::shared_ptr<ControlQueue> execQueue) {

    shared_ptr<Dmp> dmp = dynamic_pointer_cast<Dmp>(traj);
    construct(dmp, execQueue, suppressMessages);

}

void DMPExecutor::construct(std::shared_ptr<Dmp> traj, std::shared_ptr<ControlQueue> execQueue, int suppressMessages) {
    /*


    ControlQueue* controlQueue;

    std::vector<double>* vec_y;

    */

    this->isCartesian = traj->isCartesian();
    this->controlQueue = execQueue;

    this->dmp = traj;
    this->ac = 0.0;
    this->vecYs = arma::vec(1);
    this->stepSize = execQueue->getTimeStep();

    this->dmpCoeffs = traj->getDmpCoeffs();
    this->baseDef = traj->getDmpBase();

    this->tau = traj->getTau(); this->az = traj->getAz(); this->bz = traj->getBz(); this->ax = traj->getAx(); this->gs = traj->getG();
    this->y0s = this->currentJoints = traj->getY0(); this->dy0s = traj->getDy0(); this->ddy0s = traj->getDdy0();
    this->trajGen = new DMPTrajectoryGenerator(this->baseDef, ax, tau);

    this->axDivTau = ax / tau;
    this->oneDivTau = 1 / tau;

    this->simulate = SIMULATE_DMP;
    this->degofFreedom = y0s.n_elem;
    if(isCartesian)
        this->odeSystemSize = 3 * 3 + 1;
    else
        this->odeSystemSize = 2 * this->degofFreedom + 1;
    this->suppressMessages = suppressMessages;

    previousDesiredJoints = y0s;

    externalErrorUsing = 0;
    externalError = 0.0;
    t = 0.0;

    this->odeSystemSizeMinOne = odeSystemSize - 1;

}

void DMPExecutor::setTrajectory(std::shared_ptr<Trajectory> traj) {

    std::shared_ptr<Dmp> dmp = dynamic_pointer_cast<Dmp>(traj);
    construct(dmp, controlQueue, suppressMessages);

    vec_t.clear();
    vec_y.clear();

}

void DMPExecutor::useExternalError(int external) {
    externalErrorUsing = external;
}

void DMPExecutor::setExternalError(double error) {
    externalError = error;
}

int DMPExecutor::usesExternalError() {
    return externalErrorUsing;
}

double DMPExecutor::getExternalError() {
    return externalError;
}

double DMPExecutor::addTerm(double t, const double* currentDesiredYs, int jointNumber, std::shared_ptr<ControlQueue> queue) {
    return 0.0;
}

int DMPExecutor::func(double t, const double* y, double* f, void* params) {

    if(!isCartesian) {

        // joint / cartesian position

        // TODO: remove equation for z' and merge the first two equations
        // y' = z / tau
        // z' = 1 / tau * ( az * (bz * (g - y) - z) + f);
        // x' = -ax / tau * x
        for(int i = 0; i < odeSystemSizeMinOne; i = i + 2) {

            double yPlusOne = y[i + 1];

            int currentSystem = (int) (i / 2);
            f[i] = yPlusOne * oneDivTau;
            double g = gs(currentSystem);
            arma::vec currentCoeffs = dmpCoeffs.at(currentSystem);

            if(t <= (dmp->getTmax() - 1)) {

                double addTerm = trajGen->evaluateByCoefficientsSingleNonExponential(y[odeSystemSizeMinOne], currentCoeffs);
                f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne) + addTerm)  + this->addTerm(t, y, currentSystem, controlQueue);
            } else {
                f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne));
            }

        }

    } else {

        vec vecOrientationY(3);
        vec vecF0(3);
        vec vecExtAdd(3);
        for(int i = 0, dim = 0; i < odeSystemSizeMinOne; i = i + 3, ++dim) {
            vecOrientationY(dim) = y[i + 2];
            int currentSystem = (int) (i / 3);
            arma::vec currentCoeffs = dmpCoeffs.at(currentSystem);
            vecF0(dim) = trajGen->evaluateByCoefficientsSingleNonExponential(y[odeSystemSizeMinOne], currentCoeffs);
            vecExtAdd(dim) =  this->addTerm(t, y, currentSystem, controlQueue);
        }

        vec nextDEta(3);
        if(t <= (dmp->getTmax() - 1))
            nextDEta = oneDivTau * (az * (2.0 * bz * log(qG * currentQ.inverse()) - vecOrientationY) + vecF0);
        else
            nextDEta = oneDivTau * az * (2.0 * bz * log(qG * currentQ.inverse()) - vecOrientationY);


        // cartesian position and orientation
        for(int i = 0; i < odeSystemSizeMinOne; i = i + 3) {

            double yPlusOne = y[i + 1];
            int currentSystem = (int) (i / 3);
            f[i] = yPlusOne * oneDivTau;
            double g = gs(currentSystem);
            arma::vec currentCoeffs = dmpCoeffs.at(currentSystem);


            if(t <= (dmp->getTmax() - 1)) {

                double addTerm = trajGen->evaluateByCoefficientsSingleNonExponential(y[odeSystemSizeMinOne], currentCoeffs);
                f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne) + addTerm)  + this->addTerm(t, y, currentSystem, controlQueue);

            } else {
                f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne));
            }

            f[i + 2] = nextDEta(currentSystem);

        }

    }

    if(this->simulate == EXECUTE_ROBOT) {

        if(!isCartesian) currentJoints = controlQueue->getCurrentJoints().joints;
        else {
            geometry_msgs::Pose currentPose = controlQueue->getCartesianPose();
            currentJoints(0) = currentPose.position.x;
            currentJoints(1) = currentPose.position.y;
            currentJoints(2) = currentPose.position.x;
            arma::vec currentOrient = log(tf::Quaternion(currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z, currentPose.orientation.w));
            for (int i=0; i < 3; i++) currentJoints(i+3) = currentOrient(i);

        }
        double corrector = 0.0;

        if(!usesExternalError()) {

            // include original  phase stopping
            for(int i = 0; i < degofFreedom; ++i) previousDesiredJoints[i] =  y[2 * i];
            double dist = computeDistance(previousDesiredJoints, currentJoints);

            corrector = 1.0 + ac * dist;

        } else {
            corrector = 1.0 + ac * getExternalError();
        }

        f[odeSystemSizeMinOne] = -axDivTau * y[odeSystemSizeMinOne] / corrector;

    }

    else if(this->simulate == SIMULATE_DMP) {

        // progress as usual
        f[odeSystemSizeMinOne] = - axDivTau * y[odeSystemSizeMinOne];

    }

    return GSL_SUCCESS;

}

double DMPExecutor::computeDistance(const arma::vec yDes, arma::vec yCurr) {

    /*
    double dist = 0.0;
    for(int i = 0; i < degofFreedom; ++i) {
        dist += pow( yDes[i] - yCurr[i]  , 2);
    }
    return dist;
    */
    if(!isCartesian) {
        // distance for quaternion has to be introduced if this is enabled
        arma::vec tmp = (yDes - yCurr).t() * (yDes - yCurr);
        return tmp(0);
    }
    else
        return 0;
}

int DMPExecutor::jac(double t, const double* y, double *dfdy, double* dfdt, void* params) {

    // not implemented (not required for most of the ode solvers)
    return GSL_SUCCESS;

}

t_executor_res DMPExecutor::executeTrajectory(double ac, double tStart, double tEnd, double stepSize, double tolAbsErr, double tolRelErr) {

    this->ac = ac;
    this->simulate = EXECUTE_ROBOT;
    return this->executeDMP(tStart, tEnd, stepSize, tolAbsErr, tolRelErr);

}

t_executor_res DMPExecutor::simulateTrajectory(double tStart, double tEnd, double stepSize, double tolAbsErr, double tolRelErr) {

    this->simulate = SIMULATE_DMP;
    auto begin = std::chrono::high_resolution_clock::now();

    t_executor_res ret = this->executeDMP(tStart, tEnd, stepSize, tolAbsErr, tolRelErr);

    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "(DMPExecutor) the simulation took " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() << " ns" << std::endl;

    return ret;

}

t_executor_res DMPExecutor::simulateTrajectory() {

    this->simulate = SIMULATE_DMP;
    return this->executeDMP(0, dmp->getTmax(), dmp->getStepSize(), dmp->getTolAbsErr(), dmp->getTolRelErr());

}

t_executor_res DMPExecutor::executeTrajectory() {

    this->simulate = EXECUTE_ROBOT;
    return this->executeDMP(0, dmp->getTmax(), dmp->getStepSize(), dmp->getTolAbsErr(), dmp->getTolRelErr());

}

void DMPExecutor::initializeIntegration(double tStart, double stepSize, double tolAbsErr, double tolRelErr) {

    t = tStart;

    initializeIntegrationQuat();

    vec_y.clear();
    double ys[odeSystemSize];
    vecYs = vec(odeSystemSize);

    if(!isCartesian) {
        for(int i = 0; i < (odeSystemSize - 1); i = i + 2) {
            int iHalf = (int) i / 2;
            ys[i + 0] = y0s((int) iHalf);
            ys[i + 1] = tau * dy0s((int) iHalf);
        }
    } else {
        for(int i = 0, dim = 0; i < (odeSystemSize - 3); i = i + 3, ++dim) {
            int iHalf = (int) i / 3;
//            ys[i + 0] = y0s((int) iHalf);
//            ys[i + 1] = tau * dy0s((int) iHalf);
//            ys[i + 2] = dEta0(dim);
            ys[i + 0] = y0s(dim);
            ys[i + 1] = tau * dy0s(dim);
            ys[i + 2] = dEta0(dim);

            cout<<"(DMPGE) dim "<< dim<< " val eta0 "<<Eta0(dim)<<endl;
        }
    }

    ys[odeSystemSize - 1] = 1;

    sys = {static_func, NULL, odeSystemSize, this};
    d = std::shared_ptr<gsl_odeiv2_driver>(gsl_odeiv2_driver_alloc_y_new(&sys, gsl_odeiv2_step_rkf45, stepSize, tolAbsErr, tolRelErr), gsl_delete_expression());

    this->stepSize = stepSize;

    for(int i = 0; i < odeSystemSize; ++i) {
        vecYs(i) = ys[i];
    }
}

void DMPExecutor::initializeIntegrationQuat() {

    if(t > 0.0) {
        throw "(DMPExecutor) t > 0 is not considered yet with cartesian dmp";
    }

    if(isCartesian) {

        shared_ptr<CartesianDMP> cartDmp = dynamic_pointer_cast<CartesianDMP>(dmp);
        vec eta0 = cartDmp->getEta0();
        vec eta1 = cartDmp->getEtaByIdx(1);
        vec alteredEta0 = stepSize / 2.0 * oneDivTau  * eta0;
        tf::Quaternion eta0Quat(alteredEta0(0), alteredEta0(1), alteredEta0(2), 0);
        tf::Quaternion q0 = cartDmp->getQ0();
        currentQ = q0;

        dQ0 =  eta0Quat * q0;

        double firstDt = cartDmp->getDeltaTByIdx(0);
         dEta0 = 1.0 / (firstDt * cartDmp->getTau()) * (eta1 - eta0); //his os domega0?
        //dEta0 = 1.0 / firstDt * (eta1 - eta0);
        Eta0 = eta0;

        qG = cartDmp->getQg();
    }

}

arma::vec DMPExecutor::doIntegrationStep(double ac) {

    double ys[odeSystemSize];

    for(int i = 0; i < odeSystemSize; ++i)
        ys[i] = vecYs(i);

    arma::vec retJoints(degofFreedom);
    retJoints.fill(0.0);

    if(t < dmp->getTmax()) {

        int s = gsl_odeiv2_driver_apply_fixed_step(d.get(), &t, stepSize, 1, ys);


        if (s != GSL_SUCCESS) {
            cout << "(DMPExecutor) error: driver returned " << s << endl;
            throw "(DMPExecutor) error: driver returned " + s;
        }

    }

    if(!isCartesian) {

        for(int i = 0; i < degofFreedom; ++i)
            retJoints(i) = ys[2 * i];

    } else {

        vec nextEta(3);
        for(int i = 0; i < 3; ++i) {
            retJoints(i) = ys[3 * i];
            nextEta(i) = ys[3 * i + 2];
        }
        cout<<nextEta(0)<<" "<<nextEta(1)<<" "<<nextEta(2)<<" "<<t<<endl;


        nextEta = stepSize / 2.0 * oneDivTau * nextEta;
        // tf::Quaternion nextEtaQuat(nextEta(0), nextEta(1), nextEta(2), 0.0);
        currentQ = exp(nextEta) * currentQ;

        retJoints(3) = currentQ.x(); retJoints(4) = currentQ.y(); retJoints(5) = currentQ.z(); retJoints(6) = currentQ.w();

    }
    for(int i = 0; i < odeSystemSize; ++i)
        vecYs(i) = ys[i];


    return retJoints;

}

void DMPExecutor::destroyIntegration() {

    d = std::shared_ptr<gsl_odeiv2_driver>(nullptr);

}

t_executor_res DMPExecutor::executeDMP(double tStart, double tEnd, double stepSize, double tolAbsErr, double tolRelErr) {

    // auto begin = std::chrono::high_resolution_clock::now();

    int stepCount = (tEnd - tStart) / stepSize;
    double currentTime = 0.0;

    t_executor_res ret;
    for(int i = 0; i < degofFreedom; ++i)
        ret.y.push_back(arma::vec(stepCount));

    vector<double> retT;

    if(!isCartesian)
        controlQueue->moveJoints(y0s);
    else {
         controlQueue->addCartesianPosToQueue(vectorarma2pose(&y0s));
          //controlQueue->moveCartesian(vectorarma2pose(&y0s));
    }

    initializeIntegration(0, stepSize, tolAbsErr, tolRelErr);

    vec nextJoints(degofFreedom);
    nextJoints.fill(0.0);

    // execute dmps and compute linear combination
    for(int j = 0; j < stepCount; ++j, currentTime += stepSize) {

        try {
            nextJoints = doIntegrationStep(ac);

        } catch(const char* s) {
            puts(s);
            cerr << ": stopped execution at time " << currentTime << endl;
            break;
        }

        for(int i = 0; i < degofFreedom; ++i)
            ret.y.at(i)(j) = nextJoints(i);

        if(simulate == EXECUTE_ROBOT) {
            //    auto end = std::chrono::high_resolution_clock::now();
            //    std::cout << "(DMPExecutor) the simulation took " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() * 1e-9 << " s" << std::endl;
            //    begin = end;
            controlQueue->synchronizeToControlQueue(1);
        }

        if(!isCartesian)
            controlQueue->addJointsPosToQueue(nextJoints);
        else {
            geometry_msgs::Pose newP = vectorarma2pose(&nextJoints);
            controlQueue->addCartesianPosToQueue(newP);
        }


        retT.push_back(currentTime);

    }

    ret.t = stdToArmadilloVec(retT);

    return ret;

}

int DMPExecutor::static_func(double t, const double y[], double f[], void *params) {
    return ((DMPExecutor*)params)->func(t, y, f, NULL);
}

int DMPExecutor::static_jac (double t, const double y[], double *dfdy, double dfdt[], void *params) {
    return ((DMPExecutor*)params)->jac(t, y, dfdy, dfdt, NULL);
}
