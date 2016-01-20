#include "kinematicsmodel.hpp"

namespace kukadu {

    KinematicsModel::KinematicsModel(int degOfFreedom, double cycleTime) {
        this->degOfFreedom = degOfFreedom;
        this->cycleTime = cycleTime;
    }

    int KinematicsModel::getDegOfFreedom() {
        return degOfFreedom;
    }

    double KinematicsModel::getCycleTime() {
        return cycleTime;
    }


}
