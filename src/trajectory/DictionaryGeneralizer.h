#ifndef DICTIONARYGENERALIZER
#define DICTIONARYGENERALIZER

#include <armadillo>
#include <vector>
#include <string>
#include <mutex>
#include <memory>

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
    double alpham;
	arma::vec oldCoefficients;
	arma::vec newCoefficients;
	arma::vec currentCoefficients;
	
	std::mutex switcherMutex;
	
    std::shared_ptr<LinCombDmp> dictTraj;
	
	double stepSize;
	double tolAbsErr;
	double tolRelErr;
	double tEnd;
	double ac;
	
	double currentTime;
	
	double maxRelativeToMeanDistance;
	
    std::shared_ptr<ControlQueue> simulationQueue;
    std::shared_ptr<ControlQueue> executionQueue;
	
	arma::vec currentQuery;
    arma::vec extendedQuery;
	
    t_executor_res executeGen(arma::vec query, double tEnd, double ac, double as, int simulate);
    int computeClosestT(double t, arma::vec times);

    arma::vec computeExtendedQuery(double time, int correspondingIdx, arma::vec query);
    arma::vec computeExtendedQuery(double time, arma::vec query);

    arma::vec computeNewCoefficients(Mahalanobis metric, int correspondingIdx, arma::vec query);

public:
	
    DictionaryGeneralizer(arma::vec timeCenters, arma::vec initQueryPoint, std::shared_ptr<ControlQueue> simulationQueue, std::shared_ptr<ControlQueue> executionQueue, std::string dictionaryPath, double az, double bz,
                  double stepSize, double tolAbsErr, double tolRelErr, double ac, arma::vec trajMetricWeights, double maxRelativeToMeanDistance, double as, double alpham);
	
    DictionaryGeneralizer(arma::vec timeCenters, arma::vec initQueryPoint, std::shared_ptr<ControlQueue> simulationQueue, std::shared_ptr<ControlQueue> executionQueue, std::string dictionaryPath, double az, double bz,
                  double stepSize, double tolAbsErr, double tolRelErr, double ac, double as, arma::mat metric, double maxRelativeToMeanDistance, double alpham);
	
	t_executor_res simulateTrajectory();
	t_executor_res executeTrajectory();
	
    void setTrajectory(std::shared_ptr<Trajectory> traj);
    std::shared_ptr<Trajectory> getTrajectory();
	
	int getQueryPointCount();
	int getDegOfFreedom();
	QueryPoint getQueryPointByIndex(int idx);
	
	void switchQueryPoint(arma::vec query);
    void setAs(double as);

	double getCurrentTime();
	
};

#endif
