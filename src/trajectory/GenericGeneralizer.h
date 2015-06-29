#ifndef GENERICGENERALIZER
#define GENERICGENERALIZER

#include <armadillo>
#include <vector>
#include <string>

#include "../types/jointdmp.h"
#include "../utils/types.h"

/** \brief 
 * 
 * 
 * \ingroup ControlPolicyFramework
 */
class GenericGeneralizer {

private:

public:

    virtual std::shared_ptr<JointDmp> generalizeDmp(GenericKernel* trajectoryKernel, GenericKernel* parameterKernel, arma::vec query, double beta) = 0;
	
};

#endif
