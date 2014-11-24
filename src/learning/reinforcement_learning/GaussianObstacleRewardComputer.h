#ifndef GAUSSIANOBSTACLEREWARDCOMPUTER
#define GAUSSIANOBSTACLEREWARDCOMPUTER

#include <armadillo>
#include <vector>
#include <cmath>

#include <RedundantKin.h>

#include "SampleRewardComputer.h"
#include "../../utils/types.h"

class GaussianObstacleRewardComputer : public TrajectoryBasedReward {

private:
	
	double my;
	double sigma;
	double height;
	
public:

    GaussianObstacleRewardComputer(double my, double sigma, double height, double tmax);
    arma::vec computeFun(double t);
	
};

class GraspingRewardComputer : public CostComputer {

private:
	
	arma::vec pos;
	
	double robotPos[4], fingerDir[4], palmNormal[4];
	double worldPos[4], worldFingerDir[4], worldPalmNormal[4];
	
public:
	
	GraspingRewardComputer(std::vector<double> finalJointPos);

	double computeCost(t_executor_res results);
	
};

class PouringRewardComputer : public CostComputer{

private:

    double targetWeight;

public:

    PouringRewardComputer(double targetWeight);

    double computeCost(t_executor_res results);

};

class SegmentationTestingRewardComputer : public TrajectoryBasedReward {

private:

    double height;
    double slope;

public:

    SegmentationTestingRewardComputer(double height, double slope, int degOfFreedom, double tmax);
    arma::vec computeFun(double t);

};

#endif
