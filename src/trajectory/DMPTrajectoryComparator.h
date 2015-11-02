#ifndef KUKADU_DMPTRAJECTORYCOMPARATOR_H
#define KUKADU_DMPTRAJECTORYCOMPARATOR_H

#include <memory>
#include <algorithm>
#include <armadillo>

#include "../types/DMP.h"
#include "../utils/utils.h"
#include "../types/KukaduTypes.h"
#include "../trajectory/DMPExecutor.h"
#include "../robot/PlottingControlQueue.h"

class DMPTrajectoryComparator {

private:
	
	double integrationStep;
	double tolAbsErr;
	double tolRelErr;
	double tTolerance;
	
	arma::vec degOfFreedomWeights;
	
    KUKADU_SHARED_PTR<Dmp> traj1;
    KUKADU_SHARED_PTR<Dmp> traj2;

    KUKADU_SHARED_PTR<PlottingControlQueue> simQueue;
	
    KUKADU_SHARED_PTR<ControllerResult> dmp1Result;
    KUKADU_SHARED_PTR<ControllerResult> dmp2Result;
	
	void initAll(double integrationStep, double tolAbsErr, double tolRelErr, arma::vec degOfFreedomWeights, double tTolerance);

    KUKADU_SHARED_PTR<ControllerResult> executeTrajectory(KUKADU_SHARED_PTR<Dmp> traj);
	
public:
	
    DMPTrajectoryComparator(KUKADU_SHARED_PTR<ControllerResult> res1, KUKADU_SHARED_PTR<ControllerResult> res2, arma::vec degOfFreedomWeights);
    DMPTrajectoryComparator(KUKADU_SHARED_PTR<Dmp> traject1, KUKADU_SHARED_PTR<Dmp> traject2, arma::vec degOfFreedomWeights, double integrationStep, double tolAbsErr, double tolRelErr, double tTolerance);
	
    void setTrajectories(KUKADU_SHARED_PTR<Dmp> traj1, KUKADU_SHARED_PTR<Dmp> traj2, double integrationStep, double tolAbsErr, double tolRelErr, double tTolerance);
	
	double computeDistance();
	
};

#endif
