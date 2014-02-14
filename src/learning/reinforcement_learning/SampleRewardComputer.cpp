#include "SampleRewardComputer.h"

using namespace std;
using namespace arma;

SampleRewardComputer::SampleRewardComputer(double slope) {
	this->slope = slope;
}

double SampleRewardComputer::computeFun(double t) {
	
	double val = 0.0;

	val = sin(t * 1.5);
	return val;
	
}