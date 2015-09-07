#ifndef GENDMPREINFORCER
#define GENDMPREINFORCER

#include <armadillo>
#include <vector>
#include <cfloat>
#include <memory>

#include "../../trajectory/DMPExecutor.h"
#include "CostComputer.h"
#include "DMPReinforcer.h"
#include "../../trajectory/DMPGeneralizer.h"
#include "../../robot/ControlQueue.h"
#include "../../learning/GenericKernel.h"
#include "../../types/DMP.h"
#include "../../types/QueryPoint.h"
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
	
    std::shared_ptr<DMPGeneralizer> dmpGen;
	GenericKernel* trajectoryKernel;
	GenericKernel* parameterKernel;
	arma::vec initialQueryPoint;
	arma::vec lastQueryPoint;
	
    std::shared_ptr<Dmp> lastUpdate;
    std::vector<std::shared_ptr<ControllerResult>> genResults;
	bool isFirstRolloutAfterInit;
	
	double ql;
	double qh;

    std::shared_ptr<PlottingControlQueue> simQueue;
	
	/**
	 * \brief plots feedback graphs
	 */
    void plotFeedback(std::shared_ptr<DMPGeneralizer> dmpGen, std::shared_ptr<Dmp> rollout, std::shared_ptr<ControllerResult> currentRolloutRes);

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
    GenDMPReinforcer(arma::vec initialQueryPoint, CostComputer* cost, std::shared_ptr<DMPGeneralizer> dmpGen, GenericKernel* trajectoryKernel, GenericKernel* parameterKernel, std::shared_ptr<ControlQueue> movementQueue,
			 double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);
	
    std::vector<std::shared_ptr<Dmp>> getInitialRollout();
    std::vector<std::shared_ptr<Dmp>> computeRolloutParamters();
    std::shared_ptr<Dmp> updateStep();
    std::shared_ptr<Dmp> getLastUpdate();
	
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
	
};

#endif
