#ifndef KUKADU_GENDMPREINFORCER_H
#define KUKADU_GENDMPREINFORCER_H

#include <vector>
#include <cfloat>
#include <armadillo>

#include "CostComputer.h"
#include "DMPReinforcer.h"
#include "../../types/DMP.h"
#include "../../types/QueryPoint.h"
#include "../../types/KukaduTypes.h"
#include "../../robot/ControlQueue.h"
#include "../../learning/GenericKernel.h"
#include "../../trajectory/DMPExecutor.h"
#include "../../trajectory/DMPGeneralizer.h"
#include "../../robot/PlottingControlQueue.h"

/** \brief The GenDMPReinforcer completes the DMPReinforcer implementation by reusing the DMPGeneralizer functionality
 * 
 * This class computes the next rollout by working in the low dimensional query space of the method implemented by the DMPGeneralizer.
 * Therefore, the precondition of this method are similar trajectories for nearby query points. This implementation can only handle
 * one dimensional query points.
 * \ingroup ControlPolicyFramework
 */
class GenDMPReinforcer : public DMPReinforcer {

private:

    bool isFirstRolloutAfterInit;

    double ql;
    double qh;

    GenericKernel* parameterKernel;
    GenericKernel* trajectoryKernel;

    arma::vec lastQueryPoint;
    arma::vec initialQueryPoint;
	
    KUKADU_SHARED_PTR<Dmp> lastUpdate;
    KUKADU_SHARED_PTR<DMPGeneralizer> dmpGen;
    KUKADU_SHARED_PTR<PlottingControlQueue> simQueue;

    std::vector<KUKADU_SHARED_PTR<ControllerResult> > genResults;

	/**
	 * \brief plots feedback graphs
	 */
    void plotFeedback(KUKADU_SHARED_PTR<DMPGeneralizer> dmpGen, KUKADU_SHARED_PTR<Dmp> rollout, KUKADU_SHARED_PTR<ControllerResult> currentRolloutRes);

public:

	/**
	 * \brief constructor
	 * \param initialQueryPoint desired query point for which reinforcement learning should be applied
	 * \param cost CostComputer implementation
	 * \param dmpGen DMPGeneralizer instance
	 * \param trajectoryKernel generalization kernel (see DMPGeneralizer documentation)
	 * \param parameterKernel generalization kernel (see DMPGeneralizer documentation)
	 * \param movementQueue control queue for robot execution
	 * \param ac phase stopping parameter
	 * \param dmpStepSize step size for dmp execution
	 * \param tolAbsErr absolute tolerated error for numerical approximation
	 * \param tolRelErr relative tolerated error for numerical approximation
	 */
    GenDMPReinforcer(arma::vec initialQueryPoint, CostComputer* cost, KUKADU_SHARED_PTR<DMPGeneralizer> dmpGen, GenericKernel* trajectoryKernel, GenericKernel* parameterKernel, KUKADU_SHARED_PTR<ControlQueue> movementQueue,
			 double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

    /**
     * \brief returns maximum query point
     */
    double getQMin();

    /**
     * \brief returns minimum query point
     */
    double getQMax();

    /**
     * \brief sets qMin value (can be used to increas rl speed significantly)
     */
    double setQMin(double qMin);

    /**
     * \brief sets qMax value (can be used to increas rl speed significantly)
     */
    double setQMax(double qMax);
	
    std::vector<KUKADU_SHARED_PTR<Dmp> > getInitialRollout();
    std::vector<KUKADU_SHARED_PTR<Dmp> > computeRolloutParamters();

    KUKADU_SHARED_PTR<Dmp> updateStep();
    KUKADU_SHARED_PTR<Dmp> getLastUpdate();
	
};

#endif
