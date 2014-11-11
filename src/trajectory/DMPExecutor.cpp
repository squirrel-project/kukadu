#include "DMPExecutor.h"

using namespace std;
using namespace arma;

DMPExecutor::DMPExecutor(Dmp traj) {
	
	construct(traj, 1);
	
}

DMPExecutor::DMPExecutor(Dmp traj, int suppressMessages) {
	
	construct(traj, suppressMessages);
	
}

DMPExecutor::DMPExecutor(Trajectory* traj) {
	
	Dmp dmp = *((Dmp*) traj);
	construct(dmp, suppressMessages);
	
}

void DMPExecutor::construct(Dmp traj, int suppressMessages) {
    /*


    ControlQueue* controlQueue;

    std::vector<double>* vec_y;

    */

	this->dmp = traj;
    this->ac = 0.0;
    this->vecYs = arma::vec(1);
    this->stepSize = 0.014;

    this->dmpCoeffs = traj.getDmpCoeffs();
	this->baseDef = traj.getDmpBase();

    this->tau = traj.getTau(); this->az = traj.getAz(); this->bz = traj.getBz(); this->ax = traj.getAx(); this->gs = traj.getG();
    this->y0s = this->currentJoints = traj.getY0(); this->dy0s = traj.getDy0(); this->ddy0s = traj.getDdy0();
    this->trajGen = new DMPTrajectoryGenerator(this->baseDef, ax, tau);

    this->axDivTau = ax / tau;
    this->oneDivTau = 1 / tau;

	this->simulate = SIMULATE_DMP;
    this->degofFreedom = y0s.n_elem;
	this->odeSystemSize = 2 * this->degofFreedom + 1;
	this->suppressMessages = suppressMessages;

    previousDesiredJoints = y0s;

    duration = traj.getTmax();
	externalErrorUsing = 0;
	externalError = 0.0;
	t = 0.0;

    this->durationThresh = duration;
    this->odeSystemSizeMinOne = odeSystemSize - 1;
	
}

void DMPExecutor::setTrajectory(Trajectory* traj) {
	
	Dmp dmp = *((Dmp*) traj);
	construct(dmp, suppressMessages);
	
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

double DMPExecutor::addTerm(double t, const double* currentDesiredYs, int jointNumber, ControlQueue* queue) {
    return 0.0;
}

int DMPExecutor::func(double t, const double* y, double* f, void* params) {

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
		
        if(t <= (durationThresh - 1)) {
			
            double addTerm = trajGen->evaluateByCoefficientsSingleNonExponential(y[odeSystemSizeMinOne], currentCoeffs);
            f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne) + addTerm)  + this->addTerm(t, y, i / 2, controlQueue);
			
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

double DMPExecutor::computeDistance(const arma::vec yDes, arma::vec yCurr) {

    /*
	double dist = 0.0;
	for(int i = 0; i < degofFreedom; ++i) {
		dist += pow( yDes[i] - yCurr[i]  , 2);
	}
    return dist;
    */

    arma::vec tmp = (yDes - yCurr).t() * (yDes - yCurr);
    return tmp(0);

}
     
int DMPExecutor::jac(double t, const double* y, double *dfdy, double* dfdt, void* params) {
	
	// not implemented (not required for most of the ode solvers)
	return GSL_SUCCESS;
	
}

t_executor_res DMPExecutor::executeTrajectory(double ac, double tStart, double tEnd, double stepSize, double tolAbsErr, double tolRelErr, ControlQueue* controlQueue) {

	this->ac = ac;
	this->simulate = EXECUTE_ROBOT;
	this->controlQueue = controlQueue;
	return this->executeDMP(EXECUTE_ROBOT, tStart, tEnd, stepSize, tolAbsErr, tolRelErr);
	
}

t_executor_res DMPExecutor::simulateTrajectory(double tStart, double tEnd, double stepSize, double tolAbsErr, double tolRelErr) {

	this->simulate = SIMULATE_DMP;
	this->controlQueue = NULL;
    auto begin = std::chrono::high_resolution_clock::now();

    t_executor_res ret = this->executeDMP(SIMULATE_DMP, tStart, tEnd, stepSize, tolAbsErr, tolRelErr);

    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "(DMPExecutor) the simulation took " << std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count() << " ns" << std::endl;

    return ret;

}

t_executor_res DMPExecutor::simulateTrajectory() {
	
	return this->executeDMP(SIMULATE_DMP, 0, dmp.getTmax(), dmp.getStepSize(), dmp.getTolAbsErr(), dmp.getTolRelErr());
	
}

t_executor_res DMPExecutor::executeTrajectory() {
	return this->executeDMP(EXECUTE_ROBOT, 0, dmp.getTmax(), dmp.getStepSize(), dmp.getTolAbsErr(), dmp.getTolRelErr());
}

void DMPExecutor::initializeIntegration(double tStart, double stepSize, double tolAbsErr, double tolRelErr) {

	t = tStart;
    vec_y.clear();
	double ys[odeSystemSize];
	vecYs = vec(odeSystemSize);
	
    int iHalf;
	for(int i = 0; i < (odeSystemSize - 1); i = i + 2) {
        iHalf = (int) i / 2;
        ys[i + 0] = y0s((int) iHalf);
        ys[i + 1] = tau * dy0s((int) iHalf);
	}
	
	ys[odeSystemSize - 1] = 1;
	
	sys = {static_func, NULL, odeSystemSize, this};
	d = gsl_odeiv2_driver_alloc_y_new(&sys, gsl_odeiv2_step_rkf45, stepSize, tolAbsErr, tolRelErr);
	
	this->stepSize = stepSize;
	
	for(int i = 0; i < odeSystemSize; ++i)
		vecYs(i) = ys[i];
	
}
	
arma::vec DMPExecutor::doIntegrationStep(double ac) {
	
	double ys[odeSystemSize];
	
	for(int i = 0; i < odeSystemSize; ++i)
		ys[i] = vecYs(i);
	
	arma::vec retJoints(degofFreedom);
	retJoints.fill(0.0);
	
    if(t < durationThresh) {

        int s = gsl_odeiv2_driver_apply_fixed_step(d, &t, stepSize, 1, ys);

//        cout << t << " " << ys[0] << endl;
//        getchar();
	
        if (s != GSL_SUCCESS) {
            cout << "(DMPExecutor) error: driver returned " << s << endl;
            throw "(DMPExecutor) error: driver returned " + s;
        }

    }
	
	for(int i = 0; i < degofFreedom; ++i)
		retJoints(i) = ys[2 * i];
	
	for(int i = 0; i < odeSystemSize; ++i)
        vecYs(i) = ys[i];
	
	return retJoints;
	
}

void DMPExecutor::destroyIntegration() {
	
	gsl_odeiv2_driver_free(d);
	d = NULL;

}

t_executor_res DMPExecutor::executeDMP(int simulate, double tStart, double tEnd, double stepSize, double tolAbsErr, double tolRelErr) {

    double currentTime = 0.0;

    t_executor_res ret;
    //vector<vector<double>> retY;
    vector<double>* retY = new vector<double>[degofFreedom];
    vector<double> retT;

//    DMPExecutor* exec = new DMPExecutor(dmp);
    DMPExecutor* exec = this;

    exec->initializeIntegration(0, stepSize, tolAbsErr, tolRelErr);

    // execute dmps and compute linear combination
    for(; currentTime < tEnd; currentTime += stepSize) {


        vec nextJoints(degofFreedom);
        nextJoints.fill(0.0);

        /**************actual computation of trajectory using coefficients***********/
        for(int i = 0; i < 1; ++i) {

            vec currJoints(degofFreedom);
            currJoints.fill(0.0);

            try {
                currJoints = exec->doIntegrationStep(ac);
            } catch(const char* s) {
                puts(s);
                cerr << ": stopped execution at time " << currentTime << endl;
                break;
            }

            nextJoints = currJoints;

        }
/*

        // if real robot execution and first integration step --> move to initial position
        if(isFirstIteration) {

        //    cout << "(DictionaryGeneralizer) moving to initial execution position" << endl;
            queue->moveJoints(nextJoints);
            isFirstIteration = 0;
        //    cout << "(DictionaryGeneralizer) starting trajectory execution" << endl;

        } else {

            // synchronize to control queue (maximum one joint array has to be already in there --> needed for phase stopping such that DMPExecutor does not progress to fast)
            queue->synchronizeToControlQueue(0);
            queue->addJointsPosToQueue(nextJoints);

        }
*/
        for(int i = 0; i < degofFreedom; ++i)
            retY[i].push_back(nextJoints(i));

        retT.push_back(currentTime);

    }

    for(int i = 0; i < degofFreedom; ++i)
        ret.y.push_back(stdToArmadilloVec(retY[i]));

    ret.t = stdToArmadilloVec(retT);

    return ret;

}

int DMPExecutor::static_func(double t, const double y[], double f[], void *params) {
	return ((DMPExecutor*)params)->func(t, y, f, NULL);
}

int DMPExecutor::static_jac (double t, const double y[], double *dfdy, double dfdt[], void *params) {
	return ((DMPExecutor*)params)->jac(t, y, dfdy, dfdt, NULL);
}
