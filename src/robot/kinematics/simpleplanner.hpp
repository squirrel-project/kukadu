#ifndef KUKADU_SIMPLEPLANNER_H
#define KUKADU_SIMPLEPLANNER_H

#include <vector>
#include <ros/ros.h>
#include <armadillo>
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <geometry_msgs/Pose.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include "kinematics.hpp"
#include "pathplanner.hpp"
#include "../../types/kukadutypes.hpp"
#include "../../types/kinematicsmodel.hpp"

namespace kukadu {

    class SimplePlanner : public PathPlanner {

    private:

        int degOfFreedom;

        KUKADU_SHARED_PTR<KinematicsModel> model;

        RMLPositionFlags refFlags;
        KUKADU_SHARED_PTR<ReflexxesAPI> refApi;
        KUKADU_SHARED_PTR<RMLPositionInputParameters> refInputParams;
        KUKADU_SHARED_PTR<RMLPositionOutputParameters> refOutputParams;

    public:

        SimplePlanner(KUKADU_SHARED_PTR<KinematicsModel> model);
        ~SimplePlanner();

        virtual std::vector<arma::vec> planJointTrajectory(std::vector<arma::vec> intermediateJoints);
        virtual std::vector<arma::vec> planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses);

    };

}

#endif
