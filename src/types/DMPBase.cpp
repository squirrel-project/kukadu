#include "DMPBase.h"

using namespace std;

DMPBase::DMPBase() {
	my = 0.0;
}

DMPBase::DMPBase(float my, std::vector<double> sigmas) {
	this->my = my;
	this->sigmas = sigmas;
}
	
float DMPBase::getMy() {
	return my;
}

std::vector<double> DMPBase::getSigmas() {
	return sigmas;
}

int DMPBase::operator==(DMPBase const& comp) const {
	return ( my == comp.my && compareVectorOfDoubles(sigmas, comp.sigmas) );
}