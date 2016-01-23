#include "pathplanner.hpp"

namespace kukadu {

    PathPlanner::PathPlanner(KUKADU_SHARED_PTR<Kinematics> kin) {
        this->kin = kin;
    }

    KUKADU_SHARED_PTR<Kinematics> PathPlanner::getKinematics() {
        return kin;
    }

    void PathPlanner::setCheckCollisions(bool collision) {
        checkCollision = collision;
    }

    bool PathPlanner::getCheckCollision() {
        return checkCollision;
    }

}
