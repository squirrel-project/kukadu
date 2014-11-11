#include "SampleRewardComputer.h"

using namespace std;
using namespace arma;

SampleRewardComputer::SampleRewardComputer(double slope, int degOfFreedom) : TrajectoryBasedReward(degOfFreedom) {
	this->slope = slope;
}

arma::vec SampleRewardComputer::computeFun(double t) {
	
    arma::vec retVec(1);
	double val = 0.0;

    retVec(0) = val = sin(t * 1.5);
    return retVec;
	
}
