#ifndef SAMPLEREWARDCOMPUTER
#define SAMPLEREWARDCOMPUTER

#include <armadillo>
#include <vector>
#include <cmath>

#include "TrajectoryBasedReward.h"
#include "../../utils/types.h"

class SampleRewardComputer : public TrajectoryBasedReward {

private:
	
	double tmax;
	double slope;

public:

    SampleRewardComputer(double slope, int degOfFreedom, double tmax);
	
    arma::vec computeFun(double t);
	
};

#endif
