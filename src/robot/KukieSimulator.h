#ifndef KUKIESIMULATOR
#define KUKIESIMULATOR

#include <map>
#include <string>
#include <armadillo>
#include <planning_scene_plugin/AddPrimitiveShape.h>

#include "ros/ros.h"

enum simulatorObjectType{KS_BOX, KS_SPHERE, KS_CYLINDER, KS_CONE};

class KukieSimulator {

private:

    std::string topAddObject;
    std::string topApplyForceTorque;
	
    ros::NodeHandle node;

    ros::Publisher pubAddObject;
    ros::Publisher pubApplyForceTorque;

    std::map<std::string, planning_scene_plugin::AddPrimitiveShape> objectsSet;

public:

    KukieSimulator(ros::NodeHandle node);

    void applyForceTorque(arma::vec force, arma::vec torque);
    void addPrimitiveObject(std::string id, arma::vec position, arma::vec rollPitchYaw, double mass, simulatorObjectType object, arma::vec dimensions);
    
};

#endif
