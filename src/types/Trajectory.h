#ifndef TRAJECTORY
#define TRAJECTORY

#include <vector>
#include <armadillo>
#include <memory>

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
	
    virtual std::shared_ptr<Trajectory> copy() = 0;

    virtual double getTmax() = 0;
	
};

#endif
