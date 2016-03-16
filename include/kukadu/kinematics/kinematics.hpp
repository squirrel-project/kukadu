#ifndef KUKADU_KINEMATICS_H
#define KUKADU_KINEMATICS_H

#include <vector>
#include <ros/ros.h>
#include <armadillo>
#include <geometry_msgs/Pose.h>
#include <kukadu/kinematics/constraints/constraints.hpp>
#include <kukadu/types/kukadutypes.hpp>

namespace kukadu {

    class Kinematics {

    private:

        std::vector<KUKADU_SHARED_PTR<Constraint> > Constraints;

    public:

        Kinematics();

        void addConstraint(KUKADU_SHARED_PTR<Constraint> Constraint);
        void removeConstraint(KUKADU_SHARED_PTR<Constraint> Constraint);

        int getConstraintsCount();
        int  getConstraintIdx(KUKADU_SHARED_PTR<Constraint> Constraint);

        KUKADU_SHARED_PTR<Constraint> getConstraintByIdx(int idx);

        bool checkAllConstraints(arma::vec currentState, geometry_msgs::Pose pose);

        virtual std::vector<arma::vec> computeIk(arma::vec currentJointState, const geometry_msgs::Pose& goal);
        virtual std::vector<arma::vec> computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose& goal) = 0;

        virtual geometry_msgs::Pose computeFk(std::vector<double> jointState) = 0;

    };

}

#endif
