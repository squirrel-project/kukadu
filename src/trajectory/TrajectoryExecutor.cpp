#include "TrajectoryExecutor.h"

TrajectoryExecutor::TrajectoryExecutor() : Controller("simple trajectory executor") {

}

KUKADU_SHARED_PTR<ControllerResult> TrajectoryExecutor::performAction() {
    return executeTrajectory();
}
