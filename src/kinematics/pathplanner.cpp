#include <kukadu/kinematics/pathplanner.hpp>
#include <kukadu/utils/utils.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

    PathPlanner::PathPlanner() {
    }

    void PathPlanner::setCheckCollisions(bool collision) {
        checkCollision = collision;
    }

    bool PathPlanner::getCheckCollision() {
        return checkCollision;
    }

    std::vector<arma::vec> PathPlanner::smoothJointPlan(std::vector<arma::vec> jointPlan) {

        vector<vec> smoothedPlan;

        if(jointPlan.size()) {
            vec lastUsedJoints = jointPlan.at(0);
            smoothedPlan.push_back(lastUsedJoints);
            for(vec joints : jointPlan) {
                if(computeMaxJointDistance(lastUsedJoints, joints) > 0.001) {
                    lastUsedJoints = joints;
                    smoothedPlan.push_back(lastUsedJoints);
                }
            }
        }

        return smoothedPlan;

    }

}
