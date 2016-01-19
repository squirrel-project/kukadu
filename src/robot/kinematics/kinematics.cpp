#include "kinematics.hpp"

#include "../../utils/utils.hpp"

namespace kukadu {

    geometry_msgs::Pose Kinematics::computeFk(arma::vec jointState) {
        return computeFk(armadilloToStdVec(jointState));
    }

    std::vector<arma::vec> Kinematics::computeIk(arma::vec currentJointState, const geometry_msgs::Pose &goal) {
        return computeIk(armadilloToStdVec(currentJointState), goal);
    }

}
