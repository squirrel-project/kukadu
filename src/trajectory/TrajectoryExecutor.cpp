#include "TrajectoryExecutor.h"

TrajectoryExecutor::TrajectoryExecutor() : Controller("simple trajectory executor") {

}

std::shared_ptr<ControllerResult> TrajectoryExecutor::performAction() {
    return executeTrajectory();
}
