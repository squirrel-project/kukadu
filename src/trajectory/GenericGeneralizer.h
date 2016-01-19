#ifndef KUKADU_GENERICGENERALIZER_H
#define KUKADU_GENERICGENERALIZER_H

#include <vector>
#include <string>
#include <armadillo>

#include "../utils/types.h"
#include "../types/jointdmp.h"
#include "../types/KukaduTypes.h"

namespace kukadu {

    /** \brief
     *
     *
     * \ingroup ControlPolicyFramework
     */
    class GenericGeneralizer {

    private:

    public:

        virtual KUKADU_SHARED_PTR<JointDmp> generalizeDmp(GenericKernel* trajectoryKernel, GenericKernel* parameterKernel, arma::vec query, double beta) = 0;

    };

}

#endif
