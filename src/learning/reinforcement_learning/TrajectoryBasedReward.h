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
    double step;

    arma::vec rewardsWeights;

public:

    TrajectoryBasedReward(int degOfFreedom, double tmax, double timeStep);
    TrajectoryBasedReward(int degOfFreedom, arma::vec rewardsWeights, double tmax, double timeStep);

    double computeCost(std::shared_ptr<ControllerResult> results);
	
    std::shared_ptr<ControllerResult> getOptimalTraj();
    std::shared_ptr<ControllerResult> getOptimalTraj(double tmax);
    std::shared_ptr<ControllerResult> getOptimalTraj(double tmax, int freedomIdx);
    std::shared_ptr<ControllerResult> getOptimalTraj(double tmin, double tmax, int freedomIdx);
    std::vector<arma::vec> computeFun(arma::vec t);
    void writeToFile(std::string file, double tStart, double tEnd, double stepSize);
	
    virtual arma::vec computeFun(double t) = 0;
	
};

#endif
