#include "SingleSampleTrajectory.h"

using namespace std;
using namespace arma;

SingleSampleTrajectory::SingleSampleTrajectory(arma::vec supervisedTs, std::vector<arma::vec> sampleYs) {
	this->supervisedTs = supervisedTs;
	this->sampleYs = sampleYs;
}

SingleSampleTrajectory::SingleSampleTrajectory(const SingleSampleTrajectory& copy) {
	this->supervisedTs = copy.supervisedTs;
	this->sampleYs = copy.sampleYs;
}

SingleSampleTrajectory::SingleSampleTrajectory() {
}

int SingleSampleTrajectory::getDegreesOfFreedom() const {
	return sampleYs.size();
}

int SingleSampleTrajectory::getDataPointsNum() {
	return supervisedTs.n_elem;
}

double SingleSampleTrajectory::getDataPoint(int freedomIdx, int ptIdx) {
	arma::vec sample =  getSampleYByIndex(freedomIdx);
	return sample(ptIdx);
}

double SingleSampleTrajectory::getT(int ptIdx) {
	return supervisedTs(ptIdx);
}
	
arma::vec SingleSampleTrajectory::getSupervisedTs() {
	return supervisedTs;
}

arma::vec SingleSampleTrajectory::getSampleYByIndex(int idx) {
	return sampleYs.at(idx);
}

int SingleSampleTrajectory::operator==(SingleSampleTrajectory const& comp) const {
	
	return compareArmadilloVec(supervisedTs, comp.supervisedTs) && compareVectorOfArmadillos(sampleYs, comp.sampleYs);
	
}

arma::vec SingleSampleTrajectory::getStartingPos() {
	
	vec ret(getDegreesOfFreedom());
	
	for(int i = 0; i < getDegreesOfFreedom(); ++i) {
		ret(i) = getDataPoint(i, 0);
	}
	
	return ret;
	
}

std::vector<arma::vec> SingleSampleTrajectory::getSampleYs() {
	return sampleYs;
}