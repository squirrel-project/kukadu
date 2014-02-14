#ifndef DMPBASESET
#define DMPBASESET

#include "../utils/conversion_utils.h"
#include <vector>
#include <armadillo>

class DMPBase {
	
private:
	
	float my;
	std::vector<double> sigmas;
	
public:
	
	DMPBase();
	DMPBase(float my, std::vector<double> sigmas);
	
	float getMy();
	std::vector<double> getSigmas();
	
	int operator==(DMPBase const& comp) const;
	
};

#endif