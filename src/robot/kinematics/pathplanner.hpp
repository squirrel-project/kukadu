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

        bool checkCollision;
        KUKADU_SHARED_PTR<Kinematics> kin;

    public:

        PathPlanner(KUKADU_SHARED_PTR<Kinematics> kin);

        void setCheckCollisions(bool collision);

        bool getCheckCollision();

        virtual std::vector<arma::vec> planJointTrajectory(std::vector<arma::vec> intermediateJoints) = 0;
        virtual std::vector<arma::vec> planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians) = 0;

        KUKADU_SHARED_PTR<Kinematics> getKinematics();

        static constexpr int RESULT_FAILED = 0;
        static constexpr int RESULT_SUCCESS = 1;
        static constexpr int RESULT_APPROXIMATE = 2;

    };

}

#endif
