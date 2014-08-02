#ifndef TIMEDEPDICTIONARYGENERALIZER
#define TIMEDEPDICTIONARYGENERALIZER

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
#include "DictionaryGeneralizer.h"

/** \brief 
 * 
 * 
 * \ingroup ControlPolicyFramework
 */
class TimeDepDictionaryGeneralizer : public DictionaryGeneralizer {

private:

    std::vector<double> timeCenters;
	
    t_executor_res executeGen(arma::vec query, double tEnd, double ac, double as, int simulate);

public:
    /*
    TimeDepDictionaryGeneralizer(std::vector<double> timeCenters, arma::vec initQueryPoint, ControlQueue* simulationQueue, ControlQueue* executionQueue, std::string dictionaryPath, int degOfFreedom, std::vector<double> tmpmys, std::vector<double> tmpsigmas, double az, double bz,
                  double stepSize, double tolAbsErr, double tolRelErr, double ax, double tau, double ac, arma::vec trajMetricWeights, double maxRelativeToMeanDistance, double as, double alpham);
	
    TimeDepDictionaryGeneralizer(std::vector<double> timeCenters, arma::vec initQueryPoint, ControlQueue* simulationQueue, ControlQueue* executionQueue, std::string dictionaryPath, int degOfFreedom, std::vector<double> tmpmys, std::vector<double> tmpsigmas, double az, double bz,
                  double stepSize, double tolAbsErr, double tolRelErr, double ax, double tau, double ac, double as, arma::mat metric, double maxRelativeToMeanDistance, double alpham);
	
                  */
    /*
	t_executor_res simulateTrajectory();
	t_executor_res executeTrajectory();
	
	void setTrajectory(Trajectory* traj);
	Trajectory* getTrajectory();
	
	int getQueryPointCount();
	int getDegOfFreedom();
	QueryPoint getQueryPointByIndex(int idx);
	
	void switchQueryPoint(arma::vec query);
    void setAs(double as);

	double getCurrentTime();
    */
	
};

#endif
