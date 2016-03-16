#include <kukadu/kinematics/kinematics.hpp>
#include <kukadu/utils/utils.hpp>

using namespace std;

namespace kukadu {

    Kinematics::Kinematics() {
        Constraints.clear();
    }

    void Kinematics::addConstraint(KUKADU_SHARED_PTR<Constraint> Constraint) {
        Constraints.push_back(Constraint);
    }

    void Kinematics::removeConstraint(KUKADU_SHARED_PTR<Constraint> Constraint) {
        std::remove(Constraints.begin(), Constraints.end(), Constraint);
    }

    int Kinematics::getConstraintsCount() {
        return Constraints.size();
    }

    int  Kinematics::getConstraintIdx(KUKADU_SHARED_PTR<Constraint> Constraint) {
        return std::find(Constraints.begin(), Constraints.end(), Constraint) - Constraints.begin();
    }

    KUKADU_SHARED_PTR<Constraint> Kinematics::getConstraintByIdx(int idx) {
        return Constraints.at(idx);
    }

    std::vector<arma::vec> Kinematics::computeIk(arma::vec currentJointState, const geometry_msgs::Pose &goal) {
        return computeIk(armadilloToStdVec(currentJointState), goal);
    }

    bool Kinematics::checkAllConstraints(arma::vec currentState, geometry_msgs::Pose pose) {

        for(int i = 0; i < getConstraintsCount(); ++i) {

            KUKADU_SHARED_PTR<Constraint> currRest = getConstraintByIdx(i);
            if(!currRest->stateOk(currentState, pose))
                return false;

        }

        return true;

    }

}
