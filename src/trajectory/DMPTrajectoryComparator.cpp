#include "DMPTrajectoryComparator.h"

using namespace arma;
using namespace std;

DMPTrajectoryComparator::DMPTrajectoryComparator(std::shared_ptr<Dmp> traject1, std::shared_ptr<Dmp> traject2, vec degOfFreedomWeights, double integrationStep, double tolAbsErr, double tolRelErr, double tTolerance) : traj1(traject1), traj2(traject2) {

    simQueue = std::shared_ptr<PlottingControlQueue>(new PlottingControlQueue(degOfFreedomWeights.n_elem, integrationStep));
	initAll(integrationStep, tolAbsErr, tolRelErr, degOfFreedomWeights, tTolerance);
	
	dmp1Result = executeTrajectory(traject1);
	dmp2Result = executeTrajectory(traject2);

}

DMPTrajectoryComparator::DMPTrajectoryComparator(std::shared_ptr<ControllerResult> res1, std::shared_ptr<ControllerResult> res2, vec degOfFreedomWeights) {

	dmp1Result = res1;
	dmp2Result = res2;
	this->degOfFreedomWeights = degOfFreedomWeights;

    simQueue = std::shared_ptr<PlottingControlQueue>(new PlottingControlQueue(degOfFreedomWeights.n_elem, res1->getTimes()(1) - res1->getTimes()(0)));

}

double DMPTrajectoryComparator::computeDistance() {
	
	double distance = 0.0;
    int degOfFreedom = dmp1Result->getYs().size();
	
	// do some consistency checks
    if(degOfFreedom != dmp2Result->getYs().size() || degOfFreedom != degOfFreedomWeights.n_elem) {
        cerr << "(DMPTrajectoryComparator) dimensions of weight vector and trajectory do not match (size1, size2) = (" << dmp2Result->getYs().size() << ", " << degOfFreedomWeights.n_elem << ")" << endl;
        throw "(DMPTrajectoryComparator) dimensions of weight vector and trajectory do not match (size1, size2) = (" + stringFromDouble(dmp2Result->getYs().size()) + ", " + stringFromDouble(degOfFreedomWeights.n_elem) + ")";
	}

    if(dmp1Result->getTimes().n_elem == 0 || dmp2Result->getTimes().n_elem == 0) {
        cerr << "(DMPTrajectoryComparator) one of the dmps is of length 0" << endl;
        throw "(DMPTrajectoryComparator) one of the dmps is of length 0";
    }

    double tDiff = abs(dmp1Result->getTimes()(dmp1Result->getTimes().n_elem - 1) - dmp2Result->getTimes()(dmp2Result->getTimes().n_elem - 1));
	if(tDiff > tTolerance) {
		cerr << "(DMPTrajectoryComparator) trajectories do not have same duration" << endl;
		throw "(DMPTrajectoryComparator) trajectories do not have same duration";
	}
	
	// if time difference is tolerated, compare until the smalles max_time is reached
    int trajSize = min(dmp1Result->getYs().at(0).n_elem, dmp2Result->getYs().at(0).n_elem);
	
	// another consistency check
	for(int i = 1; i < degOfFreedom; ++i) {
        if( (dmp1Result->getYs().at(0).n_elem) != dmp1Result->getYs().at(i).n_elem || dmp2Result->getYs().at(0).n_elem != dmp2Result->getYs().at(i).n_elem) {
			cerr << "(DMPTrajectoryComparator) dimensions of trajectory vectors do not match" << endl;
			throw "(DMPTrajectoryComparator) dimensions of trajectory vectors do not match";
		}
	}
	
	distance = 0.0;
	for(int i = 0;  i < degOfFreedom; ++i) {
		for(int j = 0; j < trajSize; ++j) {
            double diff = dmp1Result->getYs().at(i)(j) - dmp2Result->getYs().at(i)(j);
			distance += degOfFreedomWeights(i) * pow(diff , 2);
		}
	}
	
	return 1 / ( (double) degOfFreedom * trajSize) *  distance;

}

// TODO: add some optimiziations here (e.g. list of already executed trajectories)
void DMPTrajectoryComparator::setTrajectories(std::shared_ptr<Dmp> traj1, std::shared_ptr<Dmp> traj2, double integrationStep, double tolAbsErr, double tolRelErr, double tTolerance) {
	
    std::shared_ptr<Dmp> old1 = this->traj1;
    std::shared_ptr<Dmp> old2 = this->traj2;
	
	initAll(integrationStep, tolAbsErr, tolRelErr, degOfFreedomWeights, tTolerance);
	
	if(this->traj2 == traj1 || this->traj1 == traj2) {
        std::shared_ptr<Dmp> tmp = traj1;
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

std::shared_ptr<ControllerResult> DMPTrajectoryComparator::executeTrajectory(std::shared_ptr<Dmp> traj) {
	
    DMPExecutor dmpexec(traj, simQueue);
	
    /*
	if( abs(traj1.getTmax() - traj2.getTmax()) > tTolerance ) {
		cerr << "(DMPTrajectoryComparator) trajectories do not have same duration" << endl;
		throw "(DMPTrajectoryComparator) trajectories do not have same duration";
	}
    */
	
    return dmpexec.simulateTrajectory(0, max(traj1->getTmax(), traj2->getTmax()), integrationStep, tolAbsErr, tolRelErr);
	
}

void DMPTrajectoryComparator::initAll(double integrationStep, double tolAbsErr, double tolRelErr, vec degOfFreedomWeights, double tTolerance) {
	
	this->integrationStep = integrationStep;
	this->tolAbsErr = tolAbsErr;
	this->tolRelErr = tolRelErr;
	this->degOfFreedomWeights = degOfFreedomWeights;
	this->tTolerance = tTolerance;
	
}
