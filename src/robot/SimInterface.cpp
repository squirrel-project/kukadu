#include "SimInterface.h"

using namespace std;

SimInterface::SimInterface(int argc, char** argv, int sleepTime, ros::NodeHandle node) {
    currentTime = 0.0;
    this->sleepTime= sleepTime;

    this->argc = argc;
    this->argv = argv;
    this->node = node;
    loop_rate = new ros::Rate(1.0 / sleepTime * 1e+6);
    pubObj = node.advertise<planning_scene_plugin::AddPrimitiveShape>                ("/simulation/scene/AddPrimitiveShape",1);
    createObjClient  = node.serviceClient<planning_scene_plugin::ImportMeshFile>     ("/simulation/scene/ImportMeshFile");
    objHandleClient  = node.serviceClient<vrep_common::simRosGetObjectHandle>        ("/vrep/simRosGetObjectHandle");
    removeObjClient  = node.serviceClient<vrep_common::simRosRemoveObject>           ("/vrep/simRosRemoveObject");
    monitorObjClient = node.serviceClient<vrep_common::simRosGetObjectPose>          ("/vrep/simRosGetObjectPose");
    setPoseClient    = node.serviceClient<planning_scene_plugin::SetObjectPose>      ("/simulation/scene/SetObjectPose");
    setMaterialClient= node.serviceClient<planning_scene_plugin::SetObjectMaterial>  ("/simulation/scene/SetObjectMaterial");

    usleep(1e6);
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
void SimInterface::addPrimShape(int type,string object_id,float* position, float *orientation, float* dimen, float mass) {

    planning_scene_plugin::AddPrimitiveShape primObj;
    primObj.object_id=object_id;
    primObj.pose.position.x=position[0];
    primObj.pose.position.y=position[1];
    primObj.pose.position.z=position[2];
    primObj.pose.orientation.x=orientation[0];
    primObj.pose.orientation.y=orientation[1];
    primObj.pose.orientation.z=orientation[2];
    primObj.pose.orientation.w=orientation[3];
    primObj.mass=mass;
    vector<double> dim;
    dim.push_back(dimen[0]);
    if(type!=2) dim.push_back(dimen[1]);
    if(type==1) dim.push_back(dimen[2]);
    primObj.type=type;
    primObj.dimensions=dim;

    pubObj.publish(primObj);

    cout << "object created" << endl;

}

void SimInterface::addPrimShape(int type,string object_id) {

    planning_scene_plugin::AddPrimitiveShape primObj;
    primObj.object_id=object_id;

    primObj.pose.position.x=0.3;
    primObj.pose.position.y=0.5;
    primObj.pose.position.z=0.3;
    primObj.pose.orientation.x=0.0;
    primObj.pose.orientation.y=0.0;
    primObj.pose.orientation.z=0.0;
    primObj.pose.orientation.w=0.0;


    primObj.mass=0.4;

    if(type==1) {

        primObj.type=1;
        vector<double> dim;
        dim.push_back(0.2);
        dim.push_back(0.2);
        dim.push_back(0.2);
        primObj.dimensions=dim;
    }
    else if(type==2) {
        primObj.type=2;
        vector<double> dim;
        dim.push_back(0.2);
        primObj.dimensions=dim;
    }
    else if(type==3) {

        primObj.type=3;
        vector<double> dim;
        dim.push_back(0.2);
        dim.push_back(0.05);
        primObj.dimensions=dim;
    }
    else if(type==4) {

        primObj.type=4;
        vector<double> dim;
        dim.push_back(0.2);
        dim.push_back(0.05);
        primObj.dimensions=dim;
    }
    pubObj.publish(primObj);

    cout << "object created" << endl;
}


/*  string file_path  	The absolute path of the mesh file to import
    byte  file_format	A number, indicating the file format (OBJ=0,DXF=1,F3DS=2,ASCII_STL=3,BINARY_STL=4,COLLADA=5)
    string object_id	The name of the object in the simulation scene (if object with given name already exists, it will be removed!!!)
    goemtry_msgs/Pose pose	The target pose of the imported mesh (currently only position data is considered)
    float64 scale_factor	An optional scale factor. Can be left 0.
    float64 massThe mass of the imported object. If set to 0 a default value of 1.0kg will be used. */
void SimInterface::importMesh(string object_id, string path,float* position, float* orientation, float scale, float mass) {
    planning_scene_plugin::ImportMeshFile srv;
    srv.request.file_path=path;
    srv.request.object_id=object_id;
    srv.request.pose.position.x=position[0];
    srv.request.pose.position.y=position[1];
    srv.request.pose.position.z=position[2];
    srv.request.pose.orientation.x=orientation[0];
    srv.request.pose.orientation.y=orientation[1];
    srv.request.pose.orientation.z=orientation[2];
    srv.request.pose.orientation.w=orientation[3];
    srv.request.scale_factor=scale;
    srv.request.mass=mass;
    srv.request.file_format=3;
    createObjClient.call(srv);

    cout << "object imported" << endl;

}
void SimInterface::importMesh(string object_id, string path,float* position, float* orientation) {
    planning_scene_plugin::ImportMeshFile srv;
    srv.request.file_path=path;
    srv.request.object_id=object_id;
    srv.request.pose.position.x=position[0];
    srv.request.pose.position.y=position[1];
    srv.request.pose.position.z=position[2];
    srv.request.pose.orientation.x=orientation[0];
    srv.request.pose.orientation.y=orientation[1];
    srv.request.pose.orientation.z=orientation[2];
    srv.request.pose.orientation.w=orientation[3];
    srv.request.file_format=3;
    createObjClient.call(srv);

    cout << "object imported" << endl;

}

void SimInterface::importMesh(string object_id, string path) {
    planning_scene_plugin::ImportMeshFile srv;
    srv.request.file_path=path;
    srv.request.object_id=object_id;
    srv.request.pose.position.x=0.35;
    srv.request.pose.position.y=0.25;
    srv.request.pose.position.z=0.3;
    srv.request.file_format=3;
    srv.request.mass=0.1;
    createObjClient.call(srv);

    cout << "object imported" << endl;

}

//getting pose of the object

geometry_msgs::Pose SimInterface::getObjPose(string object_id) {

    geometry_msgs::Pose objPose;
    vrep_common::simRosGetObjectHandle handleSrv;
    handleSrv.request.objectName=object_id;
    objHandleClient.call(handleSrv);

    vrep_common::simRosGetObjectPose getPoseSrv;
    getPoseSrv.request.relativeToObjectHandle = REF_FRAME_ORIGIN;
    getPoseSrv.request.handle=handleSrv.response.handle;
    monitorObjClient.call(getPoseSrv);
    objPose = getPoseSrv.response.pose.pose;

    return objPose;
}

void SimInterface::setObjPose(string object_id, float* position, float* orientation) {
    planning_scene_plugin::SetObjectPose srv;
    srv.request.object_id=object_id;
    srv.request.pose.position.x=position[0];
    srv.request.pose.position.y=position[1];
    srv.request.pose.position.z=position[2];
    srv.request.pose.orientation.x=orientation[0];
    srv.request.pose.orientation.y=orientation[1];
    srv.request.pose.orientation.z=orientation[2];
    srv.request.pose.orientation.w=orientation[3];

    setPoseClient.call(srv);
}

void SimInterface::removeObj(string object_id) {
    cout << "removing object "<<object_id<<endl;
    vrep_common::simRosGetObjectHandle handleSrv;
    vrep_common::simRosRemoveObject remObj, remObjVis;

    char str[object_id.size() + 1];
    memcpy(str, object_id.c_str(), object_id.size() + 1);
    strcat(str,"_visible");

    handleSrv.request.objectName=str;
    objHandleClient.call(handleSrv);
    remObjVis.request.handle=handleSrv.response.handle;
    removeObjClient.call(remObjVis);

    handleSrv.request.objectName=object_id;
    objHandleClient.call(handleSrv);
    remObj.request.handle=handleSrv.response.handle;
    removeObjClient.call(remObj);

    cout << "object "<<object_id<<" removed"<<endl;
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

void SimInterface::setObjMaterial(string object_id,string material_id) {
    planning_scene_plugin::SetObjectMaterial srv;
    srv.request.material_id=material_id;
    srv.request.object_id=object_id;

    setMaterialClient.call(srv);
    cout<<"Material of object: "<<object_id<<" changed to "<<material_id<<endl;

}
