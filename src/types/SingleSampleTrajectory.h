#ifndef SINGLESAMPLETRAJECTORY
#define SINGLESAMPLETRAJECTORY

#include "Trajectory.h"
#include "../utils/types.h"
#include "../utils/conversion_utils.h"
#include <vector>
#include <armadillo>

class SingleSampleTrajectory : public Trajectory {
	
private:
	
	arma::vec supervisedTs;
	std::vector<arma::vec> sampleYs;
	
public:
	
	SingleSampleTrajectory(arma::vec supervisedTs, std::vector<arma::vec> sampleYs);
	SingleSampleTrajectory(const SingleSampleTrajectory& copy);
	SingleSampleTrajectory();
	
	int getDegreesOfFreedom() const;
	int getDataPointsNum();
	
	double getT(int ptIdx);
	double getDataPoint(int freedomIdx, int ptIdx);
	
	arma::vec getStartingPos();
	arma::vec getSupervisedTs();
	arma::vec getSampleYByIndex(int idx);
	std::vector<arma::vec> getSampleYs();
	
	virtual std::vector<arma::vec> getCoefficients() = 0;
	virtual void setCoefficients(std::vector<arma::vec> coeffs) = 0;
	
	int operator==(SingleSampleTrajectory const& comp) const;
	
	virtual Trajectory* copy() = 0;
	
};

#endif