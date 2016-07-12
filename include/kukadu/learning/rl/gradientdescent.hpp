#ifndef KUKADU_GRADIENTDESCENT_H
#define KUKADU_GRADIENTDESCENT_H

#include <vector>
#include <cstdlib>
#include <utility>
#include <iostream>
#include <armadillo>
#include <kukadu/learning/rl/costcomputer.hpp>
#include <kukadu/learning/rl/generalreinforcer.hpp>
#include <kukadu/utils/types.hpp>
#include <kukadu/types/trajectory.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/robot/arm/controlqueue.hpp>
#include <kukadu/types/dictionarytrajectory.hpp>
#include <kukadu/control/trajectoryexecutor.hpp>

namespace kukadu {

    class GradientDescent : public GeneralReinforcer {

    private:

        int updateNum;
        int updatesPerRollout;
        int importanceSamplingCount;

        double explorationSigma;

        TrajectoryExecutor* trajEx;

        kukadu_mersenne_twister generator;

        std::vector<double> sigmas;
        std::vector<kukadu_normal_distribution> normals;
        std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp;
        std::vector<std::pair <double, KUKADU_SHARED_PTR<Trajectory> > > sampleHistory;

        void construct(std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, std::vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

    public:

        GradientDescent(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, double explorationSigma, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);
        GradientDescent(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, std::vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

        KUKADU_SHARED_PTR<Trajectory> updateStep();

        std::vector<KUKADU_SHARED_PTR<Trajectory> > getInitialRollout();
        std::vector<KUKADU_SHARED_PTR<Trajectory> > computeRolloutParamters();

    };

}

#endif
