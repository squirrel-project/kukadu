#ifndef KUKADU_DMPREWARDCOMPUTER_H
#define KUKADU_DMPREWARDCOMPUTER_H

#include <cmath>
#include <vector>
#include <armadillo>
#include <kukadu/learning/rl/costcomputer.hpp>
#include <kukadu/utils/types.hpp>
#include <kukadu/utils/utils.hpp>
#include <kukadu/learning/rl/trajectorybasedreward.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/robot/arm/plottingcontrolqueue.hpp>

namespace kukadu {

    class DmpRewardComputer : public TrajectoryBasedReward {

    private:

        double az;
        double bz;
        double timeStep;

        std::string file;

        KUKADU_SHARED_PTR<ControllerResult> executionResult;

        int binaryTimeSearch(arma::vec times, double t);

    public:

        DmpRewardComputer(std::string file, double az, double bz, double timeStep, int degOfFreedom, double tmax, double step);

        arma::vec computeFun(double t);

    };

}

#endif
