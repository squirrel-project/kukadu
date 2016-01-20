#include "simpleplanner.hpp"

#include "../../utils/utils.hpp"

using namespace std;
using namespace ros;
using namespace arma;

namespace kukadu {

    SimplePlanner::SimplePlanner(KUKADU_SHARED_PTR<KinematicsModel> model) {

        this->model = model;

        degOfFreedom = model->getDegOfFreedom();

        refApi = KUKADU_SHARED_PTR<ReflexxesAPI>(new ReflexxesAPI(model->getDegOfFreedom(), 1.0 / model->getCycleTime()));
        refInputParams = KUKADU_SHARED_PTR<RMLPositionInputParameters>(new RMLPositionInputParameters(model->getDegOfFreedom()));
        refOutputParams = KUKADU_SHARED_PTR<RMLPositionOutputParameters>(new RMLPositionOutputParameters(model->getDegOfFreedom()));

        for(int i = 0; i < degOfFreedom; ++i) {
            refInputParams->MaxJerkVector->VecData[i] = RAD2DEG(0.001);
            refInputParams->MaxAccelerationVector->VecData[i] = RAD2DEG(0.001);
            refInputParams->MaxVelocityVector->VecData[i] = RAD2DEG(0.001);
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

                    refInputParams->TargetPositionVector->VecData[i] = RAD2DEG(nextJointPos(i));
                    refInputParams->CurrentPositionVector->VecData[i] = RAD2DEG(currentJointPos(i));

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
                        next(k) = DEG2RAD(refOutputParams->NewPositionVector->VecData[k]);

                    returnedTrajectory.push_back(next);

                }

            }

        }

        return returnedTrajectory;

    }

    std::vector<arma::vec> SimplePlanner::planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses) {

    }

}
