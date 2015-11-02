#ifndef KUKADU_GAUSSIANOBSTACLEREWARDCOMPUTER_H
#define KUKADU_GAUSSIANOBSTACLEREWARDCOMPUTER_H

#include <cmath>
#include <vector>
#include <armadillo>
#include <RedundantKin.h>

#include "../../utils/types.h"
#include "SampleRewardComputer.h"
#include "../../types/KukaduTypes.h"

class GaussianObstacleRewardComputer : public TrajectoryBasedReward {

private:
	
	double my;
	double sigma;
	double height;
	
public:

    GaussianObstacleRewardComputer(double my, double sigma, double height, double tmax, double stepSize);
    arma::vec computeFun(double t);
	
};

class SimpleGaussianObstacleRewardComputer : public TrajectoryBasedReward {

private:

    double my;
    double sigma;
    double height;

public:

    SimpleGaussianObstacleRewardComputer(double my, double sigma, double height, double tmax, double stepSize);
    arma::vec computeFun(double t);

};

class GraspingRewardComputer : public CostComputer {

private:
	
	arma::vec pos;
	
	double robotPos[4], fingerDir[4], palmNormal[4];
	double worldPos[4], worldFingerDir[4], worldPalmNormal[4];
	
public:
	
	GraspingRewardComputer(std::vector<double> finalJointPos);

    double computeCost(KUKADU_SHARED_PTR<ControllerResult> results);
	
};

class PouringRewardComputer : public CostComputer{

private:

    double targetWeight;

public:

    PouringRewardComputer(double targetWeight);

    double computeCost(KUKADU_SHARED_PTR<ControllerResult> results);

};

class SegmentationTestingRewardComputer : public TrajectoryBasedReward {

private:

    double height;
    double slope;

public:

    SegmentationTestingRewardComputer(double height, double slope, int degOfFreedom, double tmax, double stepSize);
    arma::vec computeFun(double t);

};

#endif
