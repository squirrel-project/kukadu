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
	
    int degOfFreedom;
	double tmax;
	double slope;

    arma::vec rewardsWeights;

public:

    TrajectoryBasedReward(int degOfFreedom);
    TrajectoryBasedReward(int degOfFreedom, arma::vec rewardsWeights);

	double computeCost(t_executor_res results);
	
    t_executor_res getOptimalTraj(double tmax, int freedomIdx);
    t_executor_res getOptimalTraj(double tmin, double tmax, int freedomIdx);
    std::vector<arma::vec> computeFun(arma::vec t);
    void writeToFile(std::string file, double tStart, double tEnd, double stepSize);
	
    virtual arma::vec computeFun(double t) = 0;
	
};

#endif
