#include "restriction.hpp"

#include "../../../utils/utils.hpp"

using namespace std;

namespace kukadu {

    MoveItRestriction::MoveItRestriction(robot_model::RobotModelPtr robotModel, planning_scene::PlanningScenePtr planningScene, KUKADU_SHARED_PTR<robot_model::JointModelGroup> modelGroup) {

        this->planningScene = planningScene;
        this->modelGroup = modelGroup;
        this->robotModel = robotModel;

    }

    bool MoveItRestriction::stateOk(arma::vec joint, geometry_msgs::Pose cartPose) {

        moveit::core::RobotState state(robotModel);
        double jointStateArray[joint.n_elem];
        for(int i = 0; i < joint.n_elem; ++i)
            jointStateArray[i] = joint[i];
        state.setJointGroupPositions(modelGroup.get(), jointStateArray);
        state.update();
        return planningScene->isStateColliding(state, modelGroup->getName());

    }

}
