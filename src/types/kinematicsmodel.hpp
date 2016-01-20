#ifndef KUKADU_KINMODEL
#define KUKADU_KINMODEL

#include <vector>
#include <armadillo>

#include "../types/kukadutypes.hpp"
#include "../utils/conversion_utils.hpp"

namespace kukadu {

    class KinematicsModel {

    private:

        int degOfFreedom;
        double cycleTime;

    public:

        KinematicsModel(int degOfFreedom, double cycleTime);

        int getDegOfFreedom();

        double getCycleTime();

    };

}

#endif
