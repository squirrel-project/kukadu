#include "TrajectoryBasedReward.h"

using namespace std;
using namespace arma;

TrajectoryBasedReward::TrajectoryBasedReward(int degOfFreedom) {

    this->degOfFreedom = degOfFreedom;
    rewardsWeights = vec(degOfFreedom);
    rewardsWeights.fill(1.0);

}

TrajectoryBasedReward::TrajectoryBasedReward(int degOfFreedom, arma::vec rewardsWeights) {

    this->degOfFreedom = degOfFreedom;
    this->rewardsWeights = rewardsWeights;

}

double TrajectoryBasedReward::computeCost(t_executor_res results) {

	int tCount = results.t.n_elem;
	tmax = results.t(tCount - 1);
	
    double reward = 0.0;

    vector<vec> funVals = computeFun(results.t);
    for(int i = 0; i < degOfFreedom; ++i) {
        vec y = results.y.at(i);

        vec diffVec = funVals.at(i) - y;
        vec rewardVec = diffVec.t() * diffVec;
        reward += rewardsWeights(i) *  rewardVec(0);

    }

//    cout << "(TrajectoryBasedReward) reward: " << reward << endl;

    reward = reward / (tCount * sum(rewardsWeights));
	reward = 1.0 / exp(sqrt(reward));
	
	return reward;

}

void TrajectoryBasedReward::writeToFile(std::string file, double tStart, double tEnd, double stepSize) {

    ofstream outFile;
    outFile.open(file);

    for(; tStart < tEnd; tStart += stepSize) {
        arma::vec currentY = computeFun(tStart);
        outFile << tStart << "\t" << currentY.t() << endl;
    }

    outFile.close();

}

t_executor_res TrajectoryBasedReward::getOptimalTraj(double tmax, int freedomIdx) {
    return getOptimalTraj(0, tmax, freedomIdx);
}

t_executor_res TrajectoryBasedReward::getOptimalTraj(double tmin, double tmax, int freedomIdx) {
	
	t_executor_res ret;
	double step = 0.1;
	this->tmax = tmax;
	
    int size = (int) ( (double) (tmax - tmin) / (double) step);
	
	vec ys = vec(size);
	ret.t = vec(size);
	int i = 0;

    for(double t = tmin; i < size; t = t + step, ++i) {
        ret.t(i) = t - tmin;
        ys(i) = computeFun(t).at(freedomIdx);
	}

	ret.y.push_back(ys);
	
	return ret;
	
}

std::vector<arma::vec> TrajectoryBasedReward::computeFun(arma::vec t) {

    vector<vec> retYs;
    for(int i = 0; i < degOfFreedom; ++i)
        retYs.push_back(vec(t.n_elem));

	for(int i = 0; i < t.n_elem; ++i) {
        vec funVal = computeFun(t(i));
        // very inefficient --> switch to arma::mat
        for(int j = 0; j < degOfFreedom; ++j) {
            vec yVec = retYs.at(j);
            yVec(i) = funVal(j);
            retYs.at(j) = yVec;
        }
	}

    return retYs;

}
