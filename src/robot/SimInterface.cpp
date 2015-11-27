#include "SimInterface.h"

using namespace std;

SimInterface::SimInterface(ros::NodeHandle node) {

    this->node = node;
    pubObj = node.advertise<planning_scene_plugin::AddPrimitiveShape>                ("/simulation/scene/AddPrimitiveShape",1);
    createObjClient  = node.serviceClient<planning_scene_plugin::ImportMeshFile>     ("/simulation/scene/ImportMeshFile");
    objHandleClient  = node.serviceClient<vrep_common::simRosGetObjectHandle>        ("/vrep/simRosGetObjectHandle");
    removeObjClient  = node.serviceClient<vrep_common::simRosRemoveObject>           ("/vrep/simRosRemoveObject");
    monitorObjClient = node.serviceClient<vrep_common::simRosGetObjectPose>          ("/vrep/simRosGetObjectPose");
    setPoseClient    = node.serviceClient<planning_scene_plugin::SetObjectPose>      ("/simulation/scene/SetObjectPose");
    setMaterialClient= node.serviceClient<planning_scene_plugin::SetObjectMaterial>  ("/simulation/scene/SetObjectMaterial");

    ros::Rate s(1);
    s.sleep();

}

/*
/simulation/scene/AddPrimitiveShape

Adds a primitive shape to the current planning scene. The shape will be inserted at the given pose, with given name.
Also a mass parameter can be set. Default value is 1.0. The origin of the shape is always the center of the shape.

The 'type' parameter specifies the type of the required shape. Supported shape types are SPHERE (type: 1), BOX (type: 2),
CYLINDER (type: 3) and CONE (type: 4).

The 'dimensions' parameter specifies the required dimensions, based on given shape type:

  BOX 		- dimensions: [SIZE_X, SIZE_Y, SIZE_Z]
  SPHERE 	- dimensions: [RADIUS]
  CYLINDER 	- dimensions: [CYLINDER_HEIGHT, CYLINDER_RADIUS]
  CONE	   	- dimensions: [CONE_HEIGHT, CONE_RADIUS] */
//void SimInterface::addPrimShape(string shape_id, float *dim)
void SimInterface::addPrimShape(int type, string object_id, std::vector<double> position, std::vector<double> orientation, std::vector<double> dimen, double mass) {

    planning_scene_plugin::AddPrimitiveShape primObj;
    primObj.object_id = object_id;
    primObj.pose.position.x = position.at(0);
    primObj.pose.position.y = position.at(1);
    primObj.pose.position.z = position.at(2);
    primObj.pose.orientation.x = orientation.at(0);
    primObj.pose.orientation.y = orientation.at(1);
    primObj.pose.orientation.z = orientation.at(2);
    primObj.pose.orientation.w = orientation.at(3);
    primObj.mass = mass;
    vector<double> dim;
    dim.push_back(dimen.at(0));
    if(type != 2) dim.push_back(dimen.at(1));
    if(type == 1) dim.push_back(dimen.at(2));
    primObj.type = type;
    primObj.dimensions = dim;

    pubObj.publish(primObj);

}

void SimInterface::addPrimShape(int type,string object_id) {

    planning_scene_plugin::AddPrimitiveShape primObj;
    primObj.object_id = object_id;

    primObj.pose.position.x = 0.3;
    primObj.pose.position.y = 0.5;
    primObj.pose.position.z = 0.3;
    primObj.pose.orientation.x = 0.0;
    primObj.pose.orientation.y = 0.0;
    primObj.pose.orientation.z = 0.0;
    primObj.pose.orientation.w = 0.0;

    primObj.mass = 0.4;

    if(type == 1) {

        primObj.type = 1;
        vector<double> dim;
        dim.push_back(0.2);
        dim.push_back(0.2);
        dim.push_back(0.2);
        primObj.dimensions = dim;
    }
    else if(type == 2) {
        primObj.type = 2;
        vector<double> dim;
        dim.push_back(0.2);
        primObj.dimensions = dim;
    }
    else if(type == 3) {

        primObj.type = 3;
        vector<double> dim;
        dim.push_back(0.2);
        dim.push_back(0.05);
        primObj.dimensions = dim;
    }
    else if(type == 4) {

        primObj.type = 4;
        vector<double> dim;
        dim.push_back(0.2);
        dim.push_back(0.05);
        primObj.dimensions = dim;
    }
    pubObj.publish(primObj);

}


/*  string file_path  	The absolute path of the mesh file to import
    byte  file_format	A number, indicating the file format (OBJ=0,DXF=1,F3DS=2,ASCII_STL=3,BINARY_STL=4,COLLADA=5)
    string object_id	The name of the object in the simulation scene (if object with given name already exists, it will be removed!!!)
    goemtry_msgs/Pose pose	The target pose of the imported mesh (currently only position data is considered)
    float64 scale_factor	An optional scale factor. Can be left 0.
    float64 massThe mass of the imported object. If set to 0 a default value of 1.0kg will be used. */
void SimInterface::importMesh(string object_id, string path, std::vector<double> position, std::vector<double> orientation, float scale, float mass) {

    planning_scene_plugin::ImportMeshFile srv;
    srv.request.file_path = path;
    srv.request.object_id = object_id;
    srv.request.pose.position.x = position.at(0);
    srv.request.pose.position.y = position.at(1);
    srv.request.pose.position.z = position.at(2);
    srv.request.pose.orientation.x = orientation.at(0);
    srv.request.pose.orientation.y = orientation.at(1);
    srv.request.pose.orientation.z = orientation.at(2);
    srv.request.pose.orientation.w = orientation.at(3);
    srv.request.scale_factor = scale;
    srv.request.mass = mass;
    srv.request.file_format = 3;
    createObjClient.call(srv);

}
void SimInterface::importMesh(string object_id, string path, std::vector<double> position, std::vector<double> orientation) {
    planning_scene_plugin::ImportMeshFile srv;
    srv.request.file_path = path;
    srv.request.object_id = object_id;
    srv.request.pose.position.x = position.at(0);
    srv.request.pose.position.y = position.at(1);
    srv.request.pose.position.z = position.at(2);
    srv.request.pose.orientation.x = orientation.at(0);
    srv.request.pose.orientation.y = orientation.at(1);
    srv.request.pose.orientation.z = orientation.at(2);
    srv.request.pose.orientation.w = orientation.at(3);
    srv.request.file_format = 3;
    createObjClient.call(srv);

}

void SimInterface::importMesh(string object_id, string path) {

    planning_scene_plugin::ImportMeshFile srv;
    srv.request.file_path = path;
    srv.request.object_id = object_id;
    srv.request.pose.position.x = 0.35;
    srv.request.pose.position.y = 0.25;
    srv.request.pose.position.z = 0.3;
    srv.request.file_format = 3;
    srv.request.mass = 0.1;
    createObjClient.call(srv);

}

//getting pose of the object
geometry_msgs::Pose SimInterface::getObjPose(string object_id) {

    geometry_msgs::Pose objPose;
    vrep_common::simRosGetObjectHandle handleSrv;
    handleSrv.request.objectName = object_id;
    objHandleClient.call(handleSrv);

    vrep_common::simRosGetObjectPose getPoseSrv;
    getPoseSrv.request.relativeToObjectHandle = REF_FRAME_ORIGIN;
    getPoseSrv.request.handle = handleSrv.response.handle;
    monitorObjClient.call(getPoseSrv);
    objPose = getPoseSrv.response.pose.pose;

    return objPose;

}

void SimInterface::setObjPose(string object_id, std::vector<double> position, std::vector<double> orientation) {
    planning_scene_plugin::SetObjectPose srv;
    srv.request.object_id = object_id;
    srv.request.pose.position.x = position.at(0);
    srv.request.pose.position.y = position.at(1);
    srv.request.pose.position.z = position.at(2);
    srv.request.pose.orientation.x = orientation.at(0);
    srv.request.pose.orientation.y = orientation.at(1);
    srv.request.pose.orientation.z = orientation.at(2);
    srv.request.pose.orientation.w = orientation.at(3);

    setPoseClient.call(srv);
}

void SimInterface::removeObj(string object_id) {

    vrep_common::simRosGetObjectHandle handleSrv;
    vrep_common::simRosRemoveObject remObj, remObjVis;

    char str[object_id.size() + 1];
    memcpy(str, object_id.c_str(), object_id.size() + 1);
    strcat(str,"_visible");

    handleSrv.request.objectName = str;
    objHandleClient.call(handleSrv);
    remObjVis.request.handle = handleSrv.response.handle;
    removeObjClient.call(remObjVis);

    handleSrv.request.objectName = object_id;
    objHandleClient.call(handleSrv);
    remObj.request.handle = handleSrv.response.handle;
    removeObjClient.call(remObj);

}

/*/simulation/scene/SetObjectMaterial

Adjust the material setting of scene object with given id. Sets the current material according to the name of the
material to use. Possible values are:

 highFrictionMaterial
 lowFrictionMaterial
 noFrictionMaterial
 bulletMaterial_sticky_special
 rest_stack_grasp_material

    Service type:
    planning_scene_plugin/SetObjectMaterial

    Parameters:
    string object_id  	The name of the object in the simulation scene
    string material_id	The name of the material to use

    Example call:
    rosservice call /simulation/scene/SetObjectMaterial "object_id: 'octopus'
    material_id: 'highFrictionMaterial'"*/

void SimInterface::setObjMaterial(string object_id, sim_friction material_id) {

    string material = "";
    switch (material_id) {
    case FRICTION_HIGH:
        material = "highFrictionMaterial";
        break;
    case FRICTION_LOW:
        material = "lowFrictionMaterial";
        break;
    case FRICTION_NO:
        material = "noFrictionMaterial";
        break;
    case FRICTION_BULLET:
        material = "bulletMaterial_sticky_special";
        break;
    case FRICTION_STACKGRASP:
        material = "rest_stack_grasp_material";
        break;
    }

    planning_scene_plugin::SetObjectMaterial srv;
    srv.request.material_id = material_id;
    srv.request.object_id = object_id;

    setMaterialClient.call(srv);

}
