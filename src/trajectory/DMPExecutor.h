#ifndef DMPEXECUTOR
#define DMPEXECUTOR

#define SIMULATE_DMP 1
#define EXECUTE_ROBOT 2

#define KUKADU_EXEC_JOINT 1
#define KUKADU_EXEC_CART 2


#include <armadillo>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <memory>
#include <ros/rate.h>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

#include "../types/Trajectory.h"
#include "TrajectoryExecutor.h"
#include "../types/DMP.h"
#include "../types/CartesianDMP.h"
#include "../types/DMPBase.h"
#include "../utils/types.h"
#include "../utils/utils.h"
#include "../trajectory/DMPTrajectoryGenerator.h"
#include "../robot/ControlQueue.h"
#include "../trajectory/JointDMPLearner.h"

struct gsl_delete_expression {
    void operator()(gsl_odeiv2_driver* p) const {
        gsl_odeiv2_driver_free(p);
    }
};

/** \brief This class is responsible for dmp execution
 * 
 * The DMPExecutor computes the evolution of the dynmic movement primitives by using a numerical differential equation solver. It provides execution and simulation mode.
 * If the execution mode is selected, a control queue has to be passed to constructor.
 * \ingroup ControlPolicyFramework
 */
class DMPExecutor : public TrajectoryExecutor {

protected:

    bool isCartesian;

	double tau;
	double az;
	double bz;
	double ax;
	double ac;

    // for optimization
    double axDivTau;
    double oneDivTau;
    int odeSystemSizeMinOne;
	
	int simulate;
    long unsigned int odeSystemSize;
	int degofFreedom;
	
	int externalErrorUsing;
	int suppressMessages;

	double externalError;
	
    arma::vec currentJoints;
    arma::vec previousDesiredJoints;

	arma::vec y0s;
	arma::vec dy0s;
	arma::vec ddy0s;
	arma::vec gs;

	std::vector<arma::vec> dmpCoeffs;
	
	std::vector<DMPBase> baseDef;

    std::shared_ptr<ControlQueue> controlQueue;
	DMPTrajectoryGenerator* trajGen;
	
	std::vector<double> internalClock;
	std::vector<double> vec_t;

    std::vector<double> vec_y;
	
	gsl_odeiv2_system sys;

    std::shared_ptr<gsl_odeiv2_driver> d;
	
	arma::vec vecYs;
	
    std::shared_ptr<Dmp> dmp;
	
	double t;
	double stepSize;

    arma::vec dEta0;
    arma::vec Eta0;
    tf::Quaternion dQ0;
    tf::Quaternion qG;
    tf::Quaternion currentQ;

    double computeDistance(const arma::vec yDes, arma::vec yCurr);
	
	// needed for workaround (see here http://stackoverflow.com/questions/10687397/static-virtual-workaround-in-gsl)
	static int static_func(double t, const double y[], double f[], void *params);
	static int static_jac (double t, const double y[], double *dfdy, double dfdt[], void *params);

    t_executor_res executeDMP(double tStart, double tEnd, double stepSize, double tolAbsErr, double tolRelErr);

protected:

    virtual int func (double t, const double* y, double* f, void* params);
	int jac(double t, const double* y, double *dfdy, double* dfdt, void* params);
    virtual double addTerm(double t, const double* currentDesiredYs, int jointNumber, std::shared_ptr<ControlQueue> queue);

public:

	/**
	 * \brief constructor
	 * \param dmp the dmp that should be executed
	 */
    DMPExecutor(std::shared_ptr<Trajectory> dmp, std::shared_ptr<ControlQueue> execQueue);
    DMPExecutor(std::shared_ptr<Dmp> dmp, std::shared_ptr<ControlQueue> execQueue);
    DMPExecutor(std::shared_ptr<Dmp> dmp, std::shared_ptr<ControlQueue> execQueue, int suppressMessages);
	
    void construct(std::shared_ptr<Dmp> dmp, std::shared_ptr<ControlQueue> execQueue, int suppressMessages);
	
    void setTrajectory(std::shared_ptr<Trajectory> traj);
	
    t_executor_res simulateTrajectory(double tStart, double tEnd, double stepSize, double tolAbsErr, double tolRelErr);
    t_executor_res executeTrajectory(double ac, double tStart, double tEnd, double stepSize, double tolAbsErr, double tolRelErr);

    t_executor_res simulateTrajectory();
    t_executor_res executeTrajectory();
	
	void initializeIntegration(double tStart, double stepSize, double tolAbsErr, double tolRelErr);
    void initializeIntegrationQuat();
	void destroyIntegration();
	
	void useExternalError(int external);
	int usesExternalError();
	
	void setExternalError(double error);
	double getExternalError();
	
	arma::vec doIntegrationStep(double ac);


};

#endif
