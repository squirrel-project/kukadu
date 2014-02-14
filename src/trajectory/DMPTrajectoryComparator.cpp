#include "DMPTrajectoryComparator.h"

using namespace arma;
using namespace std;

DMPTrajectoryComparator::DMPTrajectoryComparator(Dmp traject1, Dmp traject2, vec degOfFreedomWeights, double integrationStep, double tolAbsErr, double tolRelErr, double tTolerance) : traj1(traject1), traj2(traject2) {

	initAll(integrationStep, tolAbsErr, tolRelErr, degOfFreedomWeights, tTolerance);
	
	dmp1Result = executeTrajectory(traject1);
	dmp2Result = executeTrajectory(traject2);

}

DMPTrajectoryComparator::DMPTrajectoryComparator(t_executor_res res1, t_executor_res res2, vec degOfFreedomWeights) {

	dmp1Result = res1;
	dmp2Result = res2;
	this->degOfFreedomWeights = degOfFreedomWeights;

}

double DMPTrajectoryComparator::computeDistance() {
	
	double distance = 0.0;
	int degOfFreedom = dmp1Result.y.size();
	
	// do some consistency checks
	if(degOfFreedom != dmp2Result.y.size() || degOfFreedom != degOfFreedomWeights.n_elem) {
		cerr << "(DMPTrajectoryComparator) dimensions of weight vector and trajectory do not match" << endl;
		throw "(DMPTrajectoryComparator) dimensions of weight vector and trajectory do not match";
	}
	
	double tDiff = abs(dmp1Result.t(dmp1Result.t.n_elem - 1) - dmp2Result.t(dmp2Result.t.n_elem - 1));
	if(tDiff > tTolerance) {
		cerr << "(DMPTrajectoryComparator) trajectories do not have same duration" << endl;
		throw "(DMPTrajectoryComparator) trajectories do not have same duration";
	}
	
	// if time difference is tolerated, compare until the smalles max_time is reached
	int trajSize = min(dmp1Result.y.at(0).n_elem, dmp2Result.y.at(0).n_elem);
	
	// another consistency check
	for(int i = 1; i < degOfFreedom; ++i) {
		if( (dmp1Result.y.at(0).n_elem) != dmp1Result.y.at(i).n_elem || dmp2Result.y.at(0).n_elem != dmp2Result.y.at(i).n_elem) {
			cerr << "(DMPTrajectoryComparator) dimensions of trajectory vectors do not match" << endl;
			throw "(DMPTrajectoryComparator) dimensions of trajectory vectors do not match";
		}
	}
	
	distance = 0.0;
	for(int i = 0;  i < degOfFreedom; ++i) {
		for(int j = 0; j < trajSize; ++j) {
			double diff = dmp1Result.y.at(i)(j) - dmp2Result.y.at(i)(j);
			distance += degOfFreedomWeights(i) * pow(diff , 2);
		}
	}
	
	return 1 / ( (double) degOfFreedom * trajSize) *  distance;

}

// TODO: add some optimiziations here (e.g. list of already executed trajectories)
void DMPTrajectoryComparator::setTrajectories(Dmp traj1, Dmp traj2, double integrationStep, double tolAbsErr, double tolRelErr, double tTolerance) {
	
	Dmp old1 = this->traj1;
	Dmp old2 = this->traj2;
	
	initAll(integrationStep, tolAbsErr, tolRelErr, degOfFreedomWeights, tTolerance);
	
	if(this->traj2 == traj1 || this->traj1 == traj2) {
		Dmp tmp = traj1;
		traj1 = traj2;
		traj2 = tmp;
	}
	
	this->traj1 = traj1;
	this->traj2 = traj2;
	
	if( !(old1 == traj1))
		dmp1Result = executeTrajectory(traj1);
	
	if( !(old2 == traj2))
		dmp2Result = executeTrajectory(traj2);

}

t_executor_res DMPTrajectoryComparator::executeTrajectory(Dmp traj) {
	
	DMPExecutor dmpexec(traj);
	
	if( abs(traj1.getTmax() - traj2.getTmax()) > tTolerance ) {
		cerr << "(DMPTrajectoryComparator) trajectories do not have same duration" << endl;
		throw "(DMPTrajectoryComparator) trajectories do not have same duration";
	}
	
	return dmpexec.simulateTrajectory(0, min(traj1.getTmax(),traj2.getTmax()), integrationStep, tolAbsErr, tolRelErr);
	
}

void DMPTrajectoryComparator::initAll(double integrationStep, double tolAbsErr, double tolRelErr, vec degOfFreedomWeights, double tTolerance) {
	
	this->integrationStep = integrationStep;
	this->tolAbsErr = tolAbsErr;
	this->tolRelErr = tolRelErr;
	this->degOfFreedomWeights = degOfFreedomWeights;
	this->tTolerance = tTolerance;
	
}