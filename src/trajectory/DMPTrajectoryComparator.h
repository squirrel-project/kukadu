#ifndef DMPTRAJECTORYCOMPARATOR
#define DMPTRAJECTORYCOMPARATOR

#include <algorithm>
#include <armadillo>
#include <memory>

#include "../utils/utils.h"
#include "../types/DMP.h"
#include "../trajectory/DMPExecutor.h"
#include "../robot/PlottingControlQueue.h"

class DMPTrajectoryComparator {

private:
	
	double integrationStep;
	double tolAbsErr;
	double tolRelErr;
	double tTolerance;
	
	arma::vec degOfFreedomWeights;
	
    std::shared_ptr<Dmp> traj1;
    std::shared_ptr<Dmp> traj2;

    std::shared_ptr<PlottingControlQueue> simQueue;
	
    std::shared_ptr<ControllerResult> dmp1Result;
    std::shared_ptr<ControllerResult> dmp2Result;
	
	void initAll(double integrationStep, double tolAbsErr, double tolRelErr, arma::vec degOfFreedomWeights, double tTolerance);
    std::shared_ptr<ControllerResult> executeTrajectory(std::shared_ptr<Dmp> traj);
	
public:
	
    DMPTrajectoryComparator(std::shared_ptr<Dmp> traject1, std::shared_ptr<Dmp> traject2, arma::vec degOfFreedomWeights, double integrationStep, double tolAbsErr, double tolRelErr, double tTolerance);
    DMPTrajectoryComparator(std::shared_ptr<ControllerResult> res1, std::shared_ptr<ControllerResult> res2, arma::vec degOfFreedomWeights);
	
    void setTrajectories(std::shared_ptr<Dmp> traj1, std::shared_ptr<Dmp> traj2, double integrationStep, double tolAbsErr, double tolRelErr, double tTolerance);
	
	double computeDistance();
	
};

#endif
