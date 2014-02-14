#ifndef GENERICGENERALIZER
#define GENERICGENERALIZER

#include <armadillo>
#include <vector>
#include <string>

#include "../types/DMP.h"
#include "../utils/types.h"

/** \brief 
 * 
 * 
 * \ingroup ControlPolicyFramework
 */
class GenericGeneralizer {

private:

public:

	virtual Dmp generalizeDmp(GenericKernel* trajectoryKernel, GenericKernel* parameterKernel, arma::vec query, double beta) = 0;
	
};

#endif