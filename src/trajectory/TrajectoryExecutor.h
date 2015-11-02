#ifndef KUKADU_TRAJECTORYEXECUTOR_H
#define KUKADU_TRAJECTORYEXECUTOR_H

#include <vector>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <iostream>
#include <armadillo>

#include "../utils/types.h"
#include "../types/Trajectory.h"
#include "../types/KukaduTypes.h"
#include "../manipulation/Controller.hpp"
#include "../manipulation/ControllerResult.hpp"

class TrajectoryExecutor : public Controller {

private:


public:

    TrajectoryExecutor();

    virtual KUKADU_SHARED_PTR<ControllerResult> executeTrajectory() = 0;
    virtual KUKADU_SHARED_PTR<ControllerResult> simulateTrajectory() = 0;
	
    virtual void setTrajectory(KUKADU_SHARED_PTR<Trajectory> traj) = 0;

    KUKADU_SHARED_PTR<ControllerResult> performAction();

};

#endif
