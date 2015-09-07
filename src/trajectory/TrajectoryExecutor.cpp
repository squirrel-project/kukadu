#include "TrajectoryExecutor.h"

std::shared_ptr<ControllerResult> TrajectoryExecutor::performAction() {
    return executeTrajectory();
}
