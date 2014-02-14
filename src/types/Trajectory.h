#ifndef TRAJECTORY
#define TRAJECTORY

#include <vector>
#include <armadillo>

#include "../utils/conversion_utils.h"

class Trajectory {
	
private:
	
public:
	
	Trajectory();
	Trajectory(const Trajectory& copy);
	
	virtual int getDegreesOfFreedom() const = 0;
	virtual arma::vec getStartingPos() = 0;
	
	virtual std::vector<arma::vec> getCoefficients() = 0;
	virtual void setCoefficients(std::vector<arma::vec> coeffs) = 0;
	
	int operator==(Trajectory const& comp) const;
	
	virtual Trajectory* copy() = 0;
	
};

#endif