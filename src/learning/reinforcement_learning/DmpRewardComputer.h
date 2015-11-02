#ifndef KUKADU_DMPREWARDCOMPUTER_H
#define KUKADU_DMPREWARDCOMPUTER_H

#include <cmath>
#include <vector>
#include <armadillo>

#include "CostComputer.h"
#include "../../utils/types.h"
#include "../../utils/utils.h"
#include "TrajectoryBasedReward.h"
#include "../../types/KukaduTypes.h"
#include "../../robot/PlottingControlQueue.h"

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

#endif
