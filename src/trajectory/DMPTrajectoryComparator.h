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
	
	Dmp traj1;
	Dmp traj2;

    std::shared_ptr<PlottingControlQueue> simQueue;
	
	t_executor_res dmp1Result;
	t_executor_res dmp2Result;
	
	void initAll(double integrationStep, double tolAbsErr, double tolRelErr, arma::vec degOfFreedomWeights, double tTolerance);
	t_executor_res executeTrajectory(Dmp traj);
	
public:
	
	DMPTrajectoryComparator(Dmp traject1, Dmp traject2, arma::vec degOfFreedomWeights, double integrationStep, double tolAbsErr, double tolRelErr, double tTolerance);
	DMPTrajectoryComparator(t_executor_res res1, t_executor_res res2, arma::vec degOfFreedomWeights);
	
	void setTrajectories(Dmp traj1, Dmp traj2, double integrationStep, double tolAbsErr, double tolRelErr, double tTolerance);
	
	double computeDistance();
	
};

#endif
