#include "generaldmplearner.h"

using namespace std;
using namespace arma;

void GeneralDmpLearner::construct(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, mat joints, int degFreedom) {
    this->dmpBase = dmpBase;
    this->tau = tau; this->az = az; this->bz = bz; this->ax = ax;
    this->joints = joints;
    this->degFreedom = degFreedom;
}

GeneralDmpLearner::GeneralDmpLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, mat joints) {
    this->construct(dmpBase, tau, az, bz, ax, joints, joints.n_cols - 1);
}

GeneralDmpLearner::GeneralDmpLearner(vector<double> mysDef, vector<double> sigmasDef, double az, double bz, string file) {

    // reading in file
    mat joints = readMovements(file);

    double tau = joints(joints.n_rows - 1, 0);
    double ax = -log(0.1) / tau / tau;

    int degFreedom = joints.n_cols - 1;

    vector<DMPBase> baseDef = buildDMPBase(mysDef, sigmasDef, ax, tau);

    this->construct(baseDef, tau, az, bz, ax, joints, degFreedom);

}

GeneralDmpLearner::GeneralDmpLearner(double az, double bz, std::string file) {

    vector<double> tmpmys;
    vector<double> tmpsigmas = {0.2, 0.8};
    mat joints = readMovements(file);
    int degFreedom = joints.n_cols - 1;

    tmpmys = constructDmpMys(joints);

    double tau = joints(joints.n_rows - 1, 0);
    double ax = -log(0.1) / tau / tau;

    vector<DMPBase> baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);
    this->construct(baseDef, tau, az, bz, ax, joints, degFreedom);

}

GeneralDmpLearner::GeneralDmpLearner(double az, double bz, arma::mat joints) {

    vector<double> tmpmys;
    vector<double> tmpsigmas = {0.2, 0.8};
    int degFreedom = joints.n_cols - 1;

    tmpmys = constructDmpMys(joints);

    double tau = joints(joints.n_rows - 1, 0);
    double ax = -log(0.1) / tau / tau;

    vector<DMPBase> baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);
    this->construct(baseDef, tau, az, bz, ax, joints, degFreedom);

}

std::shared_ptr<Dmp> GeneralDmpLearner::fitTrajectories() {

    int dataPointsNum = joints.n_rows;

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

    return createDmpInstance(timeVec, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax);

}

trajectory_learner_internal GeneralDmpLearner::fitTrajectory(vec time, vec y, vec dy, vec ddy) {

    int dataPointsNum = time.n_elem;

    trajectory_learner_internal ret;
    vec vec_g(dataPointsNum);
    vec_g.fill(y(dataPointsNum - 1));

    vec fity = computeFitY(time, y, dy, ddy, vec_g);

    DMPTrajectoryGenerator dmpTrajGen(dmpBase, ax, tau);
    GeneralFitter dmpGenFit(time, fity, dataPointsNum, &dmpTrajGen);
    mat designMatrix = dmpGenFit.computeDesignMatrix();
    vec dmpCoeff = dmpGenFit.computeLinFitCoefficients(designMatrix);

    ret.fity = fity;
    ret.coeff = dmpCoeff;
    ret.desMat = designMatrix;

    return ret;

}

