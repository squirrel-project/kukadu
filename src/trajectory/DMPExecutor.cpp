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
	
	this->dmp = traj;
	
	this->dmpCoeffs = traj.getDmpCoeffs();
	this->baseDef = traj.getDmpBase();
	
	this->tau = traj.getTau(); this->az = traj.getAz(); this->bz = traj.getBz(); this->ax = traj.getAx(); this->gs = traj.getG();
	this->y0s = traj.getY0(); this->dy0s = traj.getDy0(); this->ddy0s = traj.getDdy0();
	this->trajGen = new DMPTrajectoryGenerator(this->baseDef, ax, tau);

    this->axDivTau = ax / tau;
    this->oneDivTau = 1 / tau;
	
	this->simulate = SIMULATE_DMP;
	this->degofFreedom = y0s.n_elem;
	this->odeSystemSize = 2 * this->degofFreedom + 1;
	this->suppressMessages = suppressMessages;
	
	previousDesiredJoints = new double[y0s.n_elem];
	for(int i = 0; i < y0s.n_elem; ++i) previousDesiredJoints[i] = y0s(i);
	
	duration = traj.getTmax();
	externalErrorUsing = 0;
	externalError = 0.0;
	t = 0.0;

    this->durationThresh = duration + 0.2;
    this->odeSystemSizeMinOne = odeSystemSize - 1;
	
}

void DMPExecutor::setTrajectory(Trajectory* traj) {
	
	Dmp dmp = *((Dmp*) traj);
	construct(dmp, suppressMessages);
	
	vec_t.clear();
	vec_y = new std::vector<double>();
	
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

int DMPExecutor::func(double t, const double* y, double* f, void* params) {

	// y' = z / tau
	// z' = 1 / tau * ( az * (bz * (g - y) - z) + f); 
	// x' = -ax / tau * x
    for(int i = 0; i < odeSystemSizeMinOne; i = i + 2) {

        double yPlusOne = y[i + 1];
		
		int currentSystem = (int) (i / 2);
        f[i] = yPlusOne * oneDivTau;
        double g = gs(currentSystem);
        arma::vec currentCoeffs = dmpCoeffs.at(currentSystem);
		
        if(t <= durationThresh) {
			
            double addTerm = trajGen->evaluateByCoefficientsSingleNonExponential(y[odeSystemSizeMinOne], currentCoeffs);
            f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne) + addTerm);
			
		} else {
			cout << "(DMPExecutor) executing dmp over teaching duration" << endl;
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

double DMPExecutor::computeDistance(const double* yDes, float* yCurr) {

	double dist = 0.0;
	for(int i = 0; i < degofFreedom; ++i) {
		dist += pow( yDes[i] - yCurr[i]  , 2);
	}

	return dist;

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
	vec_y = new vector<double>[degofFreedom];
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
	
	int s = gsl_odeiv2_driver_apply_fixed_step(d, &t, stepSize, 1, ys);
	
	if (s != GSL_SUCCESS) {
		cout << "(DMPExecutor) error: driver returned " << s << endl;
		throw "(DMPExecutor) error: driver returned " + s;
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

	t_executor_res result;
	
	initializeIntegration(tStart, stepSize, tolAbsErr, tolRelErr);

	if(simulate == SIMULATE_DMP) {

		if(!suppressMessages)
			cout << "(DMPExecutor) starting simulation" << endl;

	} else if(simulate == EXECUTE_ROBOT) {

		cout << "(DMPExecutor) starting robot execution" << endl;

	}

    int durationSteps = (tEnd - tStart) / stepSize;

    for (int i = 0; i < durationSteps; ++i) {
		
		currentJoints = NULL;
		if(this->simulate == EXECUTE_ROBOT) {
			
			currentJoints = controlQueue->getCurrentJoints().joints;

		}
		
		vec newJoints = doIntegrationStep(ac);

		if(this->simulate == EXECUTE_ROBOT)  {

			float* moveJoints = NULL;
			moveJoints = copyJoints(currentJoints, controlQueue->getMovementDegreesOfFreedom());
			
			// move to desired position
			for(int i = 0; i < degofFreedom; ++i) {
				moveJoints[i] = newJoints(i);
			}
			
			// synchronize to control queue (maximum one joint array has to be already in there --> needed for phase stopping such that DMPExecutor does not progress to fast)
//            controlQueue->synchronizeToControlQueue(0);
			controlQueue->addJointsPosToQueue(moveJoints);

		}
		
		// simulate perfect movement
		if(this->simulate == SIMULATE_DMP) {
			currentJoints = new float[degofFreedom];
			for(int i = 0; i < degofFreedom; ++i) currentJoints[i] = newJoints.at(i);
		}

		for(int i = 0; i < degofFreedom; ++i) 
			vec_y[i].push_back(currentJoints[i]);
		
		vec_t.push_back(t);
		internalClock.push_back(vecYs[odeSystemSize - 1]);
		

	}
	
	destroyIntegration();
	
	result.t = stdToArmadilloVec(vec_t);
	
	// this can be done much more efficient!!!
	for(int i = 0; i < degofFreedom; ++i)
		result.y.push_back(stdToArmadilloVec(vec_y[i]));
	
	result.internalClock = stdToArmadilloVec(internalClock);
	return result;

}

int DMPExecutor::static_func(double t, const double y[], double f[], void *params) {
	return ((DMPExecutor*)params)->func(t, y, f, NULL);
}

int DMPExecutor::static_jac (double t, const double y[], double *dfdy, double dfdt[], void *params) {
	return ((DMPExecutor*)params)->jac(t, y, dfdy, dfdt, NULL);
}
