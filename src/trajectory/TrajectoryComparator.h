#ifndef KUKADU_TRAJECTORYCOMPARATOR
#define KUKADU_TRAJECTORYCOMPARATOR

#include "../types/Trajectory.h"

namespace kukadu {

    class TrajectoryComparator {

    public:

        virtual double computeDistance() = 0;

    };

}

#endif
