#include "trajectoryexecutor.hpp"

namespace kukadu {

    TrajectoryExecutor::TrajectoryExecutor() : Controller("simple trajectory executor") {

    }

    KUKADU_SHARED_PTR<ControllerResult> TrajectoryExecutor::performAction() {
        return executeTrajectory();
    }

}
