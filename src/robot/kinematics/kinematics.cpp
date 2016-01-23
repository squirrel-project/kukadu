#include "kinematics.hpp"

#include "../../utils/utils.hpp"

using namespace std;

namespace kukadu {

    Kinematics::Kinematics() {
        restrictions.clear();
    }

    void Kinematics::addRestriction(KUKADU_SHARED_PTR<Restriction> restriction) {
        restrictions.push_back(restriction);
    }

    void Kinematics::removeRestriction(KUKADU_SHARED_PTR<Restriction> restriction) {
        std::remove(restrictions.begin(), restrictions.end(), restriction);
    }

    int Kinematics::getRestrictionsCount() {
        return restrictions.size();
    }

    int  Kinematics::getRestrictionIdx(KUKADU_SHARED_PTR<Restriction> restriction) {
        return std::find(restrictions.begin(), restrictions.end(), restriction) - restrictions.begin();
    }

    KUKADU_SHARED_PTR<Restriction> Kinematics::getRestrictionByIdx(int idx) {
        return restrictions.at(idx);
    }

    std::vector<arma::vec> Kinematics::computeIk(arma::vec currentJointState, const geometry_msgs::Pose &goal) {
        return computeIk(armadilloToStdVec(currentJointState), goal);
    }

    bool Kinematics::checkAllRestrictions(arma::vec currentState, geometry_msgs::Pose pose) {
        /*
cout << 1 << endl;
        for(int i = 0; i < getRestrictionsCount(); ++i) {
cout << 2 << endl;
            KUKADU_SHARED_PTR<Restriction> currRest = getRestrictionByIdx(i);
cout << 3 << endl;
            if(!currRest->stateOk(currentState, pose))
                return false;
cout << 4 << endl;
        }
*/
        return true;

    }

}
