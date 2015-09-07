#ifndef DMPREWARDCOMPUTER
#define DMPREWARDCOMPUTER

#include <armadillo>
#include <vector>
#include <cmath>

#include "CostComputer.h"
#include "TrajectoryBasedReward.h"
#include "../../robot/PlottingControlQueue.h"
#include "../../utils/types.h"
#include "../../utils/utils.h"

class DmpRewardComputer : public TrajectoryBasedReward {

private:

    std::string file;
    double az;
    double bz;
    double timeStep;

    std::shared_ptr<ControllerResult> executionResult;

    int binaryTimeSearch(arma::vec times, double t);

public:

    DmpRewardComputer(std::string file, double az, double bz, double timeStep, int degOfFreedom, double tmax, double step);
    arma::vec computeFun(double t);
	
};

#endif
