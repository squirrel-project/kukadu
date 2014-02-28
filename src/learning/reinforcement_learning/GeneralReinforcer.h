#ifndef GENERALREINFORCER
#define GENERALREINFORCER

#include <armadillo>
#include <vector>
#include <iostream>
#include <thread>

#include "CostComputer.h"
#include "../../trajectory/TrajectoryExecutor.h"
#include "../../trajectory/DMPExecutor.h"
#include "../../robot/ControlQueue.h"
#include "../../types/Trajectory.h"

/** \brief The GeneralReinforcer provides a general framework for reinforcement learning.
 * 
 * It is an abstract class that implements elementary functionality that should be in common for all reinforcement learning methods (such as rollout execution).
 * A concrete implementation of the DMPReinforcer has to implement the missing parts such as the methods getInitialRollout and computeRolloutParamters.
 * \ingroup ControlPolicyFramework
 */
class GeneralReinforcer {

private:
	
	CostComputer* cost;
	ControlQueue* movementQueue;
	TrajectoryExecutor* trajEx;
	
	std::vector<Trajectory*> rollout;
	std::vector<t_executor_res> dmpResult;
	
	t_executor_res lastUpdateRes;
	
	bool isFirstIteration;
	
	double ac;
	double dmpStepSize;
	double tolAbsErr;
	double tolRelErr;
	double lastUpdateCost;
	
	std::vector<double> lastCost;
	Trajectory* lastUpdate;

public:

	/**
	 * \brief constructor
	 * \param cost CostComputer instance that computes the cost of a given rollout
	 * \param movementQueue ControlQueue instance for robot execution
	 * \param ac dmp phase stopping parameter
	 * \param dmpStepSize step size for dmp execution
	 * \param tolAbsErr absolute tolerated error for numerical approximation
	 * \param tolRelErr relative tolerated error for numerical approximation
	 */
	GeneralReinforcer(TrajectoryExecutor* trajEx, CostComputer* cost, ControlQueue* movementQueue);
	
	/**
	 * \brief returns the first rollout of the reinforcement learning algorithm
	 */
	virtual std::vector<Trajectory*> getInitialRollout() = 0;
	
	virtual Trajectory* updateStep() = 0;
	
	Trajectory* getLastUpdate();
	void setLastUpdate(Trajectory* lastUpdate);
	
	/**
	 * \brief computes the dmp parameters for the next rollout
	 */
	virtual std::vector<Trajectory*> computeRolloutParamters() = 0;
	
	std::vector<Trajectory*> getLastRolloutParameters();
	std::vector<t_executor_res> getLastExecutionResults();
	t_executor_res getLastUpdateRes();
	double getLastUpdateReward();
	
	/**
	 * \brief executes rollout. first, the trajectory is simulated and the user is asked, whether the trajectory really should be executed
	 * \param doSimulation flag whether trajectory should be simulated
	 * \param doExecution flag whether trajectory should be executed at robot
	 */
	void performRollout(int doSimulation, int doExecution);
	
	/**
	 * \brief returns true if the first iteration has not been performed yet
	 */
	bool getIsFirstIteration();
	
	/**
	 * \brief returns cost for the last executed rollout
	 */
	std::vector<double> getLastRolloutCost();
	
};

#endif