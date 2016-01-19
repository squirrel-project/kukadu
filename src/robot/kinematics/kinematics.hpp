#ifndef KUKADU_KINEMATICS_H
#define KUKADU_KINEMATICS_H

#include <ros/ros.h>
#include <armadillo>
#include <geometry_msgs/Pose.h>

namespace kukadu {

    class Kinematics {

    private:

    public:

        virtual std::vector<arma::vec> computeIk(arma::vec currentJointState, const geometry_msgs::Pose& goal);
        virtual std::vector<arma::vec> computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose& goal) = 0;

        virtual geometry_msgs::Pose computeFk(arma::vec jointState);
        virtual geometry_msgs::Pose computeFk(std::vector<double> jointState) = 0;

    };

}

#endif
