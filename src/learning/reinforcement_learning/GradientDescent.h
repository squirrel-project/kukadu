#ifndef GRADIENTDESCENT
#define GRADIENTDESCENT

#include <iostream>
#include <armadillo>
#include <vector>
#include <cstdlib>
#include <random>
#include <utility>

#include "CostComputer.h"
#include "GeneralReinforcer.h"
#include "../../utils/types.h"
#include "../../robot/ControlQueue.h"
#include "../../types/Trajectory.h"
#include "../../trajectory/TrajectoryExecutor.h"
#include "../../types/DictionaryTrajectory.h"

/** \brief The TerminalCostComputer implements the CostComputer interface
 *
 * This class implements the CostComputer in a simple way. The cost of the last rollout is inserted manually by the user to the console.
 * This method can be used for very low dimensional reinforcement learning as there a low number of rollouts is needed.
 * \ingroup ControlPolicyFramework
 */
class GradientDescent : public GeneralReinforcer {

private:

    int importanceSamplingCount;
    int updatesPerRollout;
    double explorationSigma;
    std::vector<std::shared_ptr<Trajectory>> initDmp;
    TrajectoryExecutor* trajEx;
    int updateNum;

//	std::vector<Dmp> sampleHistory;
//	std::vector<double> rewardHistory;

    std::vector<std::pair <double, std::shared_ptr<Trajectory>>> sampleHistory;

    std::mt19937 generator;
    std::vector<std::normal_distribution<double>> normals;

    std::vector<double> sigmas;

    void construct(std::vector<std::shared_ptr<Trajectory>> initDmp, std::vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, std::shared_ptr<CostComputer> cost, std::shared_ptr<ControlQueue> simulationQueue, std::shared_ptr<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

public:

    GradientDescent(std::shared_ptr<TrajectoryExecutor> trajEx, std::vector<std::shared_ptr<Trajectory>> initDmp, double explorationSigma, int updatesPerRollout, int importanceSamplingCount, std::shared_ptr<CostComputer> cost, std::shared_ptr<ControlQueue> simulationQueue, std::shared_ptr<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);
    GradientDescent(std::shared_ptr<TrajectoryExecutor> trajEx, std::vector<std::shared_ptr<Trajectory>> initDmp, std::vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, std::shared_ptr<CostComputer> cost, std::shared_ptr<ControlQueue> simulationQueue, std::shared_ptr<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

    std::vector<std::shared_ptr<Trajectory>> getInitialRollout();
    std::vector<std::shared_ptr<Trajectory>> computeRolloutParamters();
    std::shared_ptr<Trajectory> updateStep();

};

#endif
