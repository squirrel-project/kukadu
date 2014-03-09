#ifndef DICTIONARYGENERALIZER
#define DICTIONARYGENERALIZER

#include <armadillo>
#include <vector>
#include <string>
#include <mutex>

#include "../utils/types.h"
#include "../utils/utils.h"
#include "../utils/conversion_utils.h"
#include "../types/LinCombDmp.h"
#include "../trajectory/TrajectoryExecutor.h"
#include "../trajectory/DMPTrajectoryGenerator.h"
#include "../trajectory/TrajectoryDMPLearner.h"
#include "../learning/GenericKernel.h"
#include "../learning/GaussianProcessRegressor.h"
#include "../learning/LWRRegressor.h"
#include "../robot/ControlQueue.h"
#include "DMPExecutor.h"
#include "DMPGeneralizer.h"

/** \brief 
 * 
 * 
 * \ingroup ControlPolicyFramework
 */
class DictionaryGeneralizer : public TrajectoryExecutor {

private:
	
	int newQpSwitch;
    int firstTime;
	double as;
	double switchTime;
	arma::vec oldCoefficients;
	arma::vec newCoefficients;
	arma::vec currentCoefficients;
	
	std::mutex switcherMutex;
	
	LinCombDmp* dictTraj;
	
	double stepSize;
	double tolAbsErr;
	double tolRelErr;
	double tEnd;
	double ac;
	
	double currentTime;
	
	double maxRelativeToMeanDistance;
	
	ControlQueue* queue;
	
	arma::vec currentQuery;
	
	t_executor_res executeGen(arma::vec query, double tEnd, double ac, double as);

public:
	
	DictionaryGeneralizer(arma::vec initQueryPoint, ControlQueue* queue, std::string dictionaryPath, int degOfFreedom, std::vector<double> tmpmys, std::vector<double> tmpsigmas, double az, double bz,
			      double stepSize, double tolAbsErr, double tolRelErr, double ax, double tau, double ac, arma::vec trajMetricWeights, double maxRelativeToMeanDistance, double as);
	
	DictionaryGeneralizer(arma::vec initQueryPoint, ControlQueue* queue, std::string dictionaryPath, int degOfFreedom, std::vector<double> tmpmys, std::vector<double> tmpsigmas, double az, double bz,
			      double stepSize, double tolAbsErr, double tolRelErr, double ax, double tau, double ac, double as, arma::mat metric, double maxRelativeToMeanDistance);
	
	t_executor_res simulateTrajectory();
	t_executor_res executeTrajectory();
	
	void setTrajectory(Trajectory* traj);
	Trajectory* getTrajectory();
	
	int getQueryPointCount();
	int getDegOfFreedom();
	QueryPoint getQueryPointByIndex(int idx);
	
	void switchQueryPoint(arma::vec query);

	double getCurrentTime();
	
};

#endif
