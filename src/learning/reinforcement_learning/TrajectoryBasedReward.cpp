#include "TrajectoryBasedReward.h"

using namespace std;
using namespace arma;

TrajectoryBasedReward::TrajectoryBasedReward() {
}

double TrajectoryBasedReward::computeCost(t_executor_res results) {
	int tCount = results.t.n_elem;
	tmax = results.t(tCount - 1);
	
	double reward = 0.0;
	vec y = results.y.at(0);
	vec diffVec = computeFun(results.t) - y;
	vec rewardVec = diffVec.t() * diffVec;
	reward = rewardVec(0);
	
	reward = reward / tCount;

	reward = 1.0 / exp(sqrt(reward));
	
	return reward;

}

t_executor_res TrajectoryBasedReward::getOptimalTraj(double tmax) {
	
	t_executor_res ret;
	double step = 0.1;
	this->tmax = tmax;
	
	int size = (int) ( (double) tmax / (double) step);
	
	vec ys = vec(size);
	ret.t = vec(size);
	int i = 0;
	for(double t = 0.0; i < size; t = t + step, ++i) {
		 ret.t(i) = t;
		 ys(i) = computeFun(t);
	}
	ret.y.push_back(ys);
	
	return ret;
	
}

arma::vec TrajectoryBasedReward::computeFun(arma::vec t) {
	vec y(t.n_elem);
	for(int i = 0; i < t.n_elem; ++i) {
		y(i) = computeFun(t(i));
	}
	return y;
}