#include "KukieSimulator.h"

#include "../utils/utils.h"
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <planning_scene_plugin/ApplyForceAndTorque.h>

using namespace std;

KukieSimulator::KukieSimulator(ros::NodeHandle node) {

    this->node = node;

    topAddObject = "/simulation/scene/AddPrimitiveShape";
    topApplyForceTorque = "/simulation/scene/ApplyForceAndTorque";

    pubAddObject = node.advertise<planning_scene_plugin::AddPrimitiveShape>(topAddObject, 1);
    pubApplyForceTorque = node.advertise<planning_scene_plugin::ApplyForceAndTorque>(topApplyForceTorque, 1);

    usleep(1e6);

}

void KukieSimulator::applyForceTorque(arma::vec force, arma::vec torque) {

    planning_scene_plugin::ApplyForceAndTorque ft;
    ft.wrench.force.x = force(0); ft.wrench.force.y = force(1); ft.wrench.force.z = force(2);
    ft.wrench.torque.x = torque(0); ft.wrench.torque.y = torque(1); ft.wrench.torque.z = torque(2);
    pubApplyForceTorque.publish(ft);

}

void KukieSimulator::addPrimitiveObject(std::string id, arma::vec position, arma::vec rollPitchYaw, double mass, simulatorObjectType object, arma::vec dimensions) {

    planning_scene_plugin::AddPrimitiveShape newObject;

    newObject.object_id = id;

    geometry_msgs::Pose p;
    tf::Quaternion quat(rollPitchYaw(0), rollPitchYaw(1), rollPitchYaw(2));
    p.position.x = position(0); p.position.y = position(1); p.position.z = position(2);
    p.orientation.x = quat.getX(); p.orientation.y = quat.getY(); p.orientation.z = quat.getZ(); p.orientation.w = quat.getW();
    newObject.pose = p;

    newObject.mass = mass;
    newObject.type = object + 1;
    newObject.dimensions = armadilloToStdVec(dimensions);

    objectsSet.insert(std::pair<std::string, planning_scene_plugin::AddPrimitiveShape>(id, newObject));
    pubAddObject.publish(newObject);

}
