#ifndef KUKADU_COSTCOMPUTER_H
#define KUKADU_COSTCOMPUTER_H

#include <vector>
#include <armadillo>
#include <kukadu/utils/types.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/manipulation/complexcontroller.hpp>

namespace kukadu {

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
        virtual double computeCost(KUKADU_SHARED_PTR<ControllerResult> results) = 0;

    };

}

#endif
