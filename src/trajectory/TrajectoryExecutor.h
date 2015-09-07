#ifndef TRAJECTORYEXECUTOR
#define TRAJECTORYEXECUTOR

#include <armadillo>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <memory>

#include "../utils/types.h"
#include "../types/Trajectory.h"
#include "../manipulation/Controller.hpp"
#include "../manipulation/ControllerResult.hpp"

class TrajectoryExecutor : public Controller {

private:


public:

    virtual std::shared_ptr<ControllerResult> executeTrajectory() = 0;
    virtual std::shared_ptr<ControllerResult> simulateTrajectory() = 0;
	
    virtual void setTrajectory(std::shared_ptr<Trajectory> traj) = 0;

    std::shared_ptr<ControllerResult> performAction();

};

#endif
