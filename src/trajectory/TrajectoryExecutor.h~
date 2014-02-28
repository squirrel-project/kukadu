#ifndef TRAJECTORYEXECUTOR
#define TRAJECTORYEXECUTOR

#include <armadillo>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <iostream>
#include <cstdlib>

#include "../utils/types.h"
#include "../types/Trajectory.h"

class TrajectoryExecutor {

private:


public:

	virtual t_executor_res simulateTrajectory() = 0;
	virtual t_executor_res executeTrajectory() = 0;
	
	virtual void setTrajectory(Trajectory* traj) = 0;

};

#endif