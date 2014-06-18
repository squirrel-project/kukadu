#ifndef DMPEXECUTOR
#define DMPEXECUTOR

#define SIMULATE_DMP 1
#define EXECUTE_ROBOT 2

#include <armadillo>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <chrono>


#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

#include "../types/Trajectory.h"
#include "TrajectoryExecutor.h"
#include "../types/DMP.h"
#include "../types/DMPBase.h"
#include "../utils/types.h"
#include "../utils/utils.h"
#include "../trajectory/DMPTrajectoryGenerator.h"
#include "../robot/ControlQueue.h"
#include "../trajectory/TrajectoryDMPLearner.h"

/** \brief This class is responsible for dmp execution
 * 
 * The DMPExecutor computes the evolution of the dynmic movement primitives by using a numerical differential equation solver. It provides execution and simulation mode.
 * If the execution mode is selected, a control queue has to be passed to constructor.
 * \ingroup ControlPolicyFramework
 */
class DMPExecutor : public TrajectoryExecutor {

private:

	double tau;
	double az;
	double bz;
	double ax;
	double ac;

    // for optimization
    double axDivTau;
    double oneDivTau;
    double durationThresh;
    int odeSystemSizeMinOne;

	double duration;
	
	int simulate;
	int odeSystemSize;
	int degofFreedom;
	
	int externalErrorUsing;
	int suppressMessages;

	double externalError;
	double g, y0, dy0;
	
	float* currentJoints;
	double* previousDesiredJoints;

	arma::vec y0s;
	arma::vec dy0s;
	arma::vec ddy0s;
	arma::vec gs;

	std::vector<arma::vec> dmpCoeffs;
	
	std::vector<DMPBase> baseDef;

	ControlQueue* controlQueue;
	DMPTrajectoryGenerator* trajGen;
	
	std::vector<double> internalClock;
	std::vector<double> vec_t;

	std::vector<double>* vec_y;
	
	gsl_odeiv2_system sys;
	gsl_odeiv2_driver* d;
	
	arma::vec vecYs;
	
	Dmp dmp;
	
//	double* ys;
	double t;
	double stepSize;

	double computeDistance(const double* yDes, float* yCurr);
	
	// needed for workaround (see here http://stackoverflow.com/questions/10687397/static-virtual-workaround-in-gsl)
	static int static_func(double t, const double y[], double f[], void *params);
	static int static_jac (double t, const double y[], double *dfdy, double dfdt[], void *params);

	t_executor_res executeDMP(int simulate, double tStart, double tEnd, double stepSize, double tolAbsErr, double tolRelErr);

protected:

	int func (double t, const double* y, double* f, void* params);
	int jac(double t, const double* y, double *dfdy, double* dfdt, void* params);

public:

	/**
	 * \brief constructor
	 * \param dmp the dmp that should be executed
	 */
	DMPExecutor(Dmp dmp);
	DMPExecutor(Dmp dmp, int suppressMessages);
	
	DMPExecutor(Trajectory* traj);
	
	void construct(Dmp dmp, int suppressMessages);
	
	void setTrajectory(Trajectory* traj);
	
	t_executor_res simulateTrajectory(double tStart, double tEnd, double stepSize, double tolAbsErr, double tolRelErr);
	t_executor_res executeTrajectory(double ac, double tStart, double tEnd, double stepSize, double tolAbsErr, double tolRelErr, ControlQueue* controlQueue);
	
	t_executor_res simulateTrajectory();
	t_executor_res executeTrajectory();
	
	void initializeIntegration(double tStart, double stepSize, double tolAbsErr, double tolRelErr);
	void destroyIntegration();
	
	void useExternalError(int external);
	int usesExternalError();
	
	void setExternalError(double error);
	double getExternalError();
	
	arma::vec doIntegrationStep(double ac);


};

#endif
