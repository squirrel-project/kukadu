#include "TrajectoryDMPLearner.h"

using namespace arma;
using namespace std;

void TrajectoryDMPLearner::construct(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, mat joints, int degFreedom) {
	this->dmpBase = dmpBase;
	this->tau = tau; this->az = az; this->bz = bz; this->ax = ax;
	this->joints = joints;
	this->degFreedom = degFreedom;
}

TrajectoryDMPLearner::TrajectoryDMPLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, mat joints, int degFreedom) {
	this->construct(dmpBase, tau, az, bz, ax, joints, degFreedom);
}

TrajectoryDMPLearner::TrajectoryDMPLearner(vector<double> mysDef, vector<double> sigmasDef, double az, double bz, string file, int degFreedom) {
	
	// reading in file
	mat joints = readMovements(file, degFreedom + 1);
	
	double tau = joints(joints.n_rows - 1, 0);
	double ax = -log(0.1) / tau / tau;
	
	vector<DMPBase> baseDef = buildDMPBase(mysDef, sigmasDef, ax, tau);
	
	this->construct(baseDef, tau, az, bz, ax, joints, degFreedom);
	
}

Dmp TrajectoryDMPLearner::fitTrajectories() {

//	t_learned_dmp retDmp;
    int dataPointsNum = joints.n_rows;
	
	double tmax = joints(joints.n_rows - 1, 0);
	
	vec g(degFreedom);
	vec y0(degFreedom);
	vec dy0(degFreedom);
	vec ddy0(degFreedom);
	
	vector<vec> dmpCoeffs;
	vector<vec> sampleYs;
	vector<vec> fitYs;
	
	vector<mat> designMatrices;
	vec timeVec = joints.col(0);

	// retrieve all columns for different degrees of freedom
	vector<vec> trajectories;
	for(int i = 1; i <= degFreedom; ++i) {

		vec trajectory = joints.col(i);
		trajectories.push_back(trajectory);

		vec vec_dy = computeDiscreteDerivatives(timeVec, trajectory);
		vec vec_ddy = computeDiscreteDerivatives(timeVec, vec_dy);

		trajectory_learner_internal dmpRes = fitTrajectory(timeVec, trajectory, vec_dy, vec_ddy);
		vec dmpCoeff = dmpRes.coeff;
		vec fity = dmpRes.fity;

		g(i - 1) = trajectory(dataPointsNum - 1);
		y0(i - 1) = trajectory(0);
		dy0(i - 1) = vec_dy(0);
		ddy0(i - 1) = vec_ddy(0);
		dmpCoeffs.push_back(dmpCoeff);
		sampleYs.push_back(trajectory);
		fitYs.push_back(fity);
		designMatrices.push_back(dmpRes.desMat);

	}

	return Dmp(timeVec, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax);
	
}

trajectory_learner_internal TrajectoryDMPLearner::fitTrajectory(vec time, vec y, vec dy, vec ddy) {
	
	int dataPointsNum = time.n_elem;

	trajectory_learner_internal ret;
	vec vec_g(dataPointsNum);
	vec_g.fill(y(dataPointsNum - 1));

	vec fity = tau * tau * ddy - az * (bz * (vec_g - y) - tau * dy);

    DMPTrajectoryGenerator dmpTrajGen(dmpBase, ax, tau);
    GeneralFitter dmpGenFit(time, fity, dataPointsNum, &dmpTrajGen);
    mat designMatrix = dmpGenFit.computeDesignMatrix();
	vec dmpCoeff = dmpGenFit.computeLinFitCoefficients(designMatrix);

	ret.fity = fity;
	ret.coeff = dmpCoeff;
	ret.desMat = designMatrix;
	
	return ret;

}
