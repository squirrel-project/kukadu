#include "GaussianObstacleRewardComputer.h"

using namespace std;
using namespace arma;

GaussianObstacleRewardComputer::GaussianObstacleRewardComputer(double my, double sigma, double height) {
	
	this->my = my;
	this->sigma = sigma;
	this->height = height;
	
}

double GaussianObstacleRewardComputer::computeFun(double t) {
	
	// y1 = c1 * exp(- (x - o) .^ 2 / s);
	// return height * exp(- pow((t - my), 2) / sigma);
	
	double c2 = 0.2;
	double c3 = 0.2;
	
	//c2 = 0.2;
	//c3 = 0.2;
	//y1 = c1 * exp(- (x - o) .^ 2 / s) + c2 * sin(10 * x) + c1 * c3 * x;
//	return (height * exp(- pow((t - my), 2) / sigma) + c2 * sin(10.0 * t) + height * c3 * t);
	return (height * exp(- pow((t - my), 2) / sigma) + height * c3 * t);
	
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
	
	cout << "world vector: " << worldVec.t() << endl;
	cout << "ground truth: " << pos.t() << endl;
	
	return reward;
	
}