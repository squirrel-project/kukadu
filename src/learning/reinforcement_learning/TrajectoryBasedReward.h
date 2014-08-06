#ifndef TRAJECTORYBASEDREWARD
#define TRAJECTORYBASEDREWARD

#include <armadillo>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

#include "CostComputer.h"
#include "../../utils/types.h"

class TrajectoryBasedReward : public CostComputer {

private:
	
	double tmax;
	double slope;

public:

	TrajectoryBasedReward();
	double computeCost(t_executor_res results);
	
    t_executor_res getOptimalTraj(double tmax);
    t_executor_res getOptimalTraj(double tmin, double tmax);
	arma::vec computeFun(arma::vec t);
    void writeToFile(std::string file, double tStart, double tEnd, double stepSize);
	
	virtual double computeFun(double t) = 0;
	
};

#endif
