#include "GaussianObstacleRewardComputer.h"

using namespace std;
using namespace arma;

SegmentationTestingRewardComputer::SegmentationTestingRewardComputer(double height, double slope, int degOfFreedom) : TrajectoryBasedReward(degOfFreedom) {
    this->height = height;
    this->slope = slope;
}

arma::vec SegmentationTestingRewardComputer::computeFun(double t) {

    arma::vec retVec(1);
    double center = 1.0;
    double width = 1.2;
    double endGaussian = 3.0;

    if(t > endGaussian)
        retVec(0) = (height * exp(- pow((t - center), 2) / width) + 0.1 * height * slope * (t - endGaussian));
    else
        retVec(0) = height * exp(- pow((t - center), 2) / width);

    return retVec;

}

PouringRewardComputer::PouringRewardComputer(double targetWeight) {
    this->targetWeight = targetWeight;
}

double PouringRewardComputer::computeCost(t_executor_res results) {

    double weight = 0.0;
    cout << "(PouringRewardComputer) Enter measured weight: ";
    cin >> weight;

    double delta = abs(weight - targetWeight);
    double reward = 10 / delta;

    return reward;

}

GaussianObstacleRewardComputer::GaussianObstacleRewardComputer(double my, double sigma, double height) : TrajectoryBasedReward(0) {
	
	this->my = my;
	this->sigma = sigma;
	this->height = height;
	
}

arma::vec GaussianObstacleRewardComputer::computeFun(double t) {
	
    vec retVec(1);

	// y1 = c1 * exp(- (x - o) .^ 2 / s);
//    return height * exp(- pow((t - my), 2) / sigma);
	
	double c2 = 0.2;
	double c3 = 0.2;
	
	//c2 = 0.2;
	//c3 = 0.2;
	//y1 = c1 * exp(- (x - o) .^ 2 / s) + c2 * sin(10 * x) + c1 * c3 * x;
//	return (height * exp(- pow((t - my), 2) / sigma) + c2 * sin(10.0 * t) + height * c3 * t);
    retVec(0) = (height * exp(- pow((t - my), 2) / sigma) + height * c3 * t);
    return retVec;
	
	//y1 = c1 * exp(- (x - o) .^ 2 / s) + c2 * sin(10 * x) + c1 * o / 10.0 * x;
//	return (height * exp(- pow((t - my), 2) / sigma) + height * c3 / 8.0 * t);
	
}

GraspingRewardComputer::GraspingRewardComputer(std::vector<double> finalJointPos) {
	
	double measuredJointPosition[7];
	for(int i = 0; i < 7; ++i)
		measuredJointPosition[i] = finalJointPos.at(i);
	
	double rightR2WTM[4][4] = {
		{-0.7162,0.6041,0.3496,-0.4837},
		{-0.0168,0.4858,-0.8739,0.7469},
		{-0.6977,-0.6317,-0.3377,0.644},
		{0.0000,-0.0000,-0.0000,1.0000}};
		
	forwardKinEE(robotPos, fingerDir, palmNormal, measuredJointPosition);
	transformPos(worldPos, rightR2WTM, robotPos);
	
	pos = vec(3);
	for(int i = 0; i < 3; ++i)
		pos(i) = worldPos[i];
	
	this->pos = pos;
	
}

double GraspingRewardComputer::computeCost(t_executor_res results) {
	
	double rightR2WTM[4][4] = {
		{-0.7162,0.6041,0.3496,-0.4837},
		{-0.0168,0.4858,-0.8739,0.7469},
		{-0.6977,-0.6317,-0.3377,0.644},
		{0.0000,-0.0000,-0.0000,1.0000}};
		
	double measuredJointPosition[7];
	for(int i = 0; i < 7; ++i) {
		vec currDeg = results.y.at(i);
		measuredJointPosition[i] = currDeg(results.t.size() - 1);
	}
	
	forwardKinEE(robotPos, fingerDir, palmNormal, measuredJointPosition);
	transformPos(worldPos, rightR2WTM, robotPos);
	
	vec worldVec(3);
	for(int i = 0; i < 3; ++i)
		worldVec(i) = worldPos[i];
	
	vec distVec = (worldVec - pos).t() * (worldVec - pos);
	double reward = 1 / exp( distVec(0) );
	
//    cout << "world vector: " << worldVec.t() << endl;
//    cout << "ground truth: " << pos.t() << endl;
	
	return reward;
	
}
