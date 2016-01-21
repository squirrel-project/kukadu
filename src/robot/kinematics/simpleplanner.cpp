#include "simpleplanner.hpp"

#include "../../utils/utils.hpp"

using namespace std;
using namespace ros;
using namespace arma;

namespace kukadu {

    SimplePlanner::SimplePlanner(KUKADU_SHARED_PTR<ControlQueue> queue, KUKADU_SHARED_PTR<Kinematics> kin) {

        this->queue = queue;
        this->kin = kin;

        cycleTime = queue->getTimeStep();
        degOfFreedom = queue->getMovementDegreesOfFreedom();

        refApi = KUKADU_SHARED_PTR<ReflexxesAPI>(new ReflexxesAPI(degOfFreedom, 1.0 / cycleTime));
        refInputParams = KUKADU_SHARED_PTR<RMLPositionInputParameters>(new RMLPositionInputParameters(degOfFreedom));
        refOutputParams = KUKADU_SHARED_PTR<RMLPositionOutputParameters>(new RMLPositionOutputParameters(degOfFreedom));

        for(int i = 0; i < degOfFreedom; ++i) {
            // this seems to be not normal velocity but velocity normalized by time step
            refInputParams->MaxJerkVector->VecData[i] = 0.003 * cycleTime;
            refInputParams->MaxAccelerationVector->VecData[i] = 0.004 * cycleTime;
            refInputParams->MaxVelocityVector->VecData[i] = 0.002 * cycleTime;
            refInputParams->SelectionVector->VecData[i] = true;
        }

    }

    SimplePlanner::~SimplePlanner() {
        refOutputParams.reset();
        refInputParams.reset();
        refApi.reset();
    }

    std::vector<arma::vec> SimplePlanner::planJointTrajectory(std::vector<arma::vec> intermediateJoints) {

        vector<vec> returnedTrajectory;

        int intermedJointsSize = intermediateJoints.size();

        if(intermedJointsSize >= 1) {

            returnedTrajectory.push_back(intermediateJoints.at(0));

            bool firstTime = true;
            for(int j = 0; j + 1 < intermedJointsSize; ++j) {

                vec currentJointPos = intermediateJoints.at(j);
                vec nextJointPos = intermediateJoints.at(j + 1);

                for(int i = 0; i < degOfFreedom; ++i) {

                    refInputParams->TargetPositionVector->VecData[i] = nextJointPos(i);
                    refInputParams->CurrentPositionVector->VecData[i] = currentJointPos(i);

                    if(firstTime) {
                        refInputParams->CurrentVelocityVector->VecData[i] = 0.0;
                        refInputParams->CurrentAccelerationVector->VecData[i] = 0.0;
                        refInputParams->TargetVelocityVector->VecData[i] = 0.0;
                    } else {
                        // already set below
                    }

                }

                int result = ReflexxesAPI::RML_ERROR;
                while(result != ReflexxesAPI::RML_FINAL_STATE_REACHED) {

                    result = refApi->RMLPosition(*refInputParams, refOutputParams.get(), refFlags);
                    refInputParams->CurrentPositionVector = refOutputParams->NewPositionVector;
                    refInputParams->CurrentVelocityVector = refOutputParams->NewVelocityVector;
                    refInputParams->CurrentAccelerationVector = refOutputParams->NewAccelerationVector;

                    vec next(degOfFreedom);
                    for(int k = 0; k < degOfFreedom; ++k)
                        next(k) = refOutputParams->NewPositionVector->VecData[k];

                    returnedTrajectory.push_back(next);

                }

            }

        }

        return returnedTrajectory;

    }

    std::vector<arma::vec> SimplePlanner::planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses) {

        vector<vec> retJoints;
        vec currJoints = queue->getCurrentJoints().joints;
        retJoints.push_back(currJoints);

        int posesCount = intermediatePoses.size();
        for(int i = 0; i < posesCount; ++i) {
            cout << i << endl;
            vector<vec> nextIk = kin->computeIk(currJoints, intermediatePoses.at(i));
            if(nextIk.size()) {
                currJoints = nextIk.at(0);
                retJoints.push_back(nextIk.at(0));
            } else
                return vector<vec>();
        }

        retJoints.push_back(currJoints);

        return planJointTrajectory(retJoints);

    }

}
