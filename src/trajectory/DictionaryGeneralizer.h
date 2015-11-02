#ifndef KUKADU_DICTIONARYGENERALIZER_H
#define KUKADU_DICTIONARYGENERALIZER_H

#include <vector>
#include <string>
#include <armadillo>

#include "../utils/types.h"
#include "../utils/utils.h"
#include "../types/LinCombDmp.h"
#include "../types/KukaduTypes.h"
#include "../robot/ControlQueue.h"
#include "../learning/LWRRegressor.h"
#include "../trajectory/DMPExecutor.h"
#include "../utils/conversion_utils.h"
#include "../learning/GenericKernel.h"
#include "../trajectory/DMPGeneralizer.h"
#include "../trajectory/JointDMPLearner.h"
#include "../trajectory/TrajectoryExecutor.h"
#include "../trajectory/DMPTrajectoryGenerator.h"
#include "../learning/GaussianProcessRegressor.h"

/** \brief 
 * 
 * 
 * \ingroup ControlPolicyFramework
 */
class DictionaryGeneralizer : public TrajectoryExecutor {

private:
	
    int firstTime;
    int newQpSwitch;

    double ac;
	double as;
    double alpham;
    double stepSize;
    double tolAbsErr;
    double tolRelErr;
	double switchTime;
    double currentTime;
    double maxRelativeToMeanDistance;

    arma::vec currentQuery;
    arma::vec extendedQuery;
	arma::vec oldCoefficients;
	arma::vec newCoefficients;
	arma::vec currentCoefficients;
	
    kukadu_mutex switcherMutex;
	
    KUKADU_SHARED_PTR<LinCombDmp> dictTraj;
    KUKADU_SHARED_PTR<ControlQueue> executionQueue;
    KUKADU_SHARED_PTR<ControlQueue> simulationQueue;
    KUKADU_SHARED_PTR<ControllerResult> executeGen(arma::vec query, double tEnd, double ac, double as, int simulate);

    int computeClosestT(double t, arma::vec times);

    arma::vec computeExtendedQuery(double time, arma::vec query);
    arma::vec computeExtendedQuery(double time, int correspondingIdx, arma::vec query);
    arma::vec computeNewCoefficients(Mahalanobis metric, int correspondingIdx, arma::vec query);

public:
	
    DictionaryGeneralizer(arma::vec timeCenters, arma::vec initQueryPoint, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, std::string dictionaryPath, double az, double bz,
                  double stepSize, double tolAbsErr, double tolRelErr, double ac, arma::vec trajMetricWeights, double maxRelativeToMeanDistance, double as, double alpham);
	
    DictionaryGeneralizer(arma::vec timeCenters, arma::vec initQueryPoint, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, std::string dictionaryPath, double az, double bz,
                  double stepSize, double tolAbsErr, double tolRelErr, double ac, double as, arma::mat metric, double maxRelativeToMeanDistance, double alpham);
	
    void setAs(double as);
    void switchQueryPoint(arma::vec query);
    void setTrajectory(KUKADU_SHARED_PTR<Trajectory> traj);

    int getDegOfFreedom();
    int getQueryPointCount();

    double getCurrentTime();

    QueryPoint getQueryPointByIndex(int idx);

    KUKADU_SHARED_PTR<Trajectory> getTrajectory();
    KUKADU_SHARED_PTR<ControllerResult> executeTrajectory();
    KUKADU_SHARED_PTR<ControllerResult> simulateTrajectory();

};

#endif
