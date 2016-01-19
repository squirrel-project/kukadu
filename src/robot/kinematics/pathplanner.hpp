#ifndef KUKADU_PATHPLANNER_H
#define KUKADU_PATHPLANNER_H

#include <vector>
#include <armadillo>
#include <geometry_msgs/Pose.h>

#include "kinematics.hpp"
#include "../../types/kukadutypes.hpp"

namespace kukadu {

    class PathPlanner {

    private:


    public:

        virtual std::vector<arma::vec> planJointTrajectory(std::vector<arma::vec> intermediateJoints) = 0;
        virtual std::vector<arma::vec> planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses) = 0;

    };

}

#endif
