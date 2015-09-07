#ifndef COSTCOMPUTER
#define COSTCOMPUTER

#include <vector>
#include <memory>
#include <armadillo>


#include "../../utils/types.h"
#include "../../manipulation/ControllerResult.hpp"

/** \brief Interface for reinforcement learning cost function computation used by DMPReinforcer
 * 
 * This class provides the necessary interfaces for the cost function computation
 * \ingroup ControlPolicyFramework
 */
class CostComputer {

private:

public:

	/**
	 * \brief computes cost for a given dmp execution
	 * \param results measured results of the last dmp execution
	 */
    virtual double computeCost(std::shared_ptr<ControllerResult> results) = 0;
	
};

#endif
