#ifndef KUKADU_POWER_H
#define KUKADU_POWER_H

#include <vector>
#include <cstdlib>
#include <utility>
#include <iostream>
#include <armadillo>

#include "CostComputer.h"
#include "GeneralReinforcer.h"
#include "../../utils/types.h"
#include "../../types/Trajectory.h"
#include "../../types/KukaduTypes.h"
#include "../../robot/ControlQueue.h"
#include "../../trajectory/TrajectoryExecutor.h"

/** \brief The TerminalCostComputer implements the CostComputer interface
 * 
 * This class implements the CostComputer in a simple way. The cost of the last rollout is inserted manually by the user to the console.
 * This method can be used for very low dimensional reinforcement learning as there a low number of rollouts is needed.
 * \ingroup ControlPolicyFramework
 */
class PoWER : public GeneralReinforcer {

private:
	
    int updatesPerRollout;
	int importanceSamplingCount;

	double explorationSigma;

    KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx;

    std::vector<double> sigmas;
    std::vector<kukadu_normal_distribution> normals;
    std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp;
    std::vector<std::pair <double, KUKADU_SHARED_PTR<Trajectory> > > sampleHistory;
	
    kukadu_mersenne_twister generator;
	
    void construct(std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, std::vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);
	
public:

    PoWER(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, double explorationSigma, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr, unsigned seed);
    PoWER(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, std::vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr, unsigned seed);
	
    KUKADU_SHARED_PTR<Trajectory> updateStep();

    std::vector<KUKADU_SHARED_PTR<Trajectory> > getInitialRollout();
    std::vector<KUKADU_SHARED_PTR<Trajectory> > computeRolloutParamters();
	
};

#endif
