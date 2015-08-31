#ifndef SIMINTERFACE_H
#define SIMINTERFACE_H


#include <unistd.h>
#include <queue>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <thread>
#include <mutex>
#include <time.h>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64MultiArray.h>

#include "planning_scene_plugin/AddPrimitiveShape.h"
#include "planning_scene_plugin/ImportMeshFile.h"
#include "planning_scene_plugin/SetObjectPose.h"
#include "planning_scene_plugin/SetObjectColor.h"
#include "planning_scene_plugin/SetObjectMaterial.h"
#include "vrep_common/simRosGetObjectPose.h"
#include "vrep_common/simRosGetObjectHandle.h"
#include "vrep_common/simRosGetObjectPose.h"
#include "vrep_common/simRosRemoveObject.h"

#define REF_FRAME_ORIGIN 166

class SimInterface {

private:
    int argc;
    int sleepTime;

    double currentTime;

    char** argv;

    ros::NodeHandle node;
    ros::Rate* loop_rate;

    ros::Publisher pubObj;
    ros::ServiceClient createObjClient;
    ros::ServiceClient objHandleClient;
    ros::ServiceClient monitorObjClient;
    ros::ServiceClient removeObjClient;
    ros::ServiceClient setPoseClient;
    ros::ServiceClient setMaterialClient;
    ros::ServiceClient setColorClient;


public:

    enum sim_friction {
        FRICTION_HIGH, FRICTION_LOW, FRICTION_NO, FRICTION_BULLET, FRICTION_STACKGRASP
    };

    SimInterface(int argc, char** argv, int sleepTime, ros::NodeHandle node);

    void addPrimShape(int type, std::string object_id);
    void addPrimShape(int type, std::string object_id, std::vector<double> position, std::vector<double> orientation, std::vector<double> dim, double mass);

    void importMesh(std::string object_id, std::string path);
    void importMesh(std::string object_id, std:: string path, std::vector<double> position, std::vector<double> orientation);
    void importMesh(std::string object_id, std:: string path, std::vector<double> position, std::vector<double> orientation, float scale, float mass);

    void removeObj(std::string object_id);
    void setObjMaterial(std::string object_id, sim_friction material_id);
    void setObjPose(std::string object_id, std::vector<double> position, std::vector<double> orientation);

    geometry_msgs::Pose getObjPose(std::string object_id);

};

#endif // SIMINTERFACE_H
