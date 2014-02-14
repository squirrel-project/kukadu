#ifndef COSTCOMPUTER
#define COSTCOMPUTER

#include <armadillo>
#include <vector>

#include "../../utils/types.h"

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
	virtual double computeCost(t_executor_res results) = 0;
	
};

#endif