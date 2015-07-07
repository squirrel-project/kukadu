#include "VisionInterface.h"


VisionInterface::VisionInterface(int argc, char** argv, int sleepTime, ros::NodeHandle node)  {

    this->sleepTime= sleepTime;
    this->argc = argc;
    this->argv = argv;
    this->node = node;

    this->construct();

}

VisionInterface::VisionInterface(int argc, char** argv, int sleepTime, std::string cameraTag, ros::NodeHandle node){

    this->sleepTime= sleepTime;
    this->argc = argc;
    this->argv = argv;
    this->node = node;
    this->currentCameraTag = cameraTag;

    this->arTagTracker = false;
}

void VisionInterface::setArTagTracker(){
    this->arTagTracker = true;
    this->subArTag = node.subscribe(arTagTopic, 2, &VisionInterface::arTagCallback, this);

}

void VisionInterface::construct(){
    arTagTopic = "arMarker/tf";


    Eigen::Matrix4f Tm;
    Tm <<   // use inverse of octave
            -6.3153e-02, -1.0343e+00, -1.8485e-02,  7.2820e-01,
            -6.9606e-01,  5.6802e-02, -6.8370e-01,  1.5751e-01,
            5.6291e-01 , 4.3940e-02 ,-7.0540e-01 , 6.5800e-01,
            -2.8443e-15,  1.7948e-15,  2.7121e-15,  1.0000e+00;

    tfChestKin = Matrix4f2Transform(Tm);


}

void VisionInterface::arTagCallback(const tf::tfMessage& msg){

    geometry_msgs::TransformStamped t = msg.transforms.at(0);
    tf::Vector3 origin;
    origin.setValue(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    tf::Quaternion rotation (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);

    tf::Transform pose;
    pose.setOrigin(origin);
    pose.setRotation(rotation);

    pose = tfChestKin * pose;

    currentArPose.position.x = pose.getOrigin().getX();
    currentArPose.position.y = pose.getOrigin().getY();
    currentArPose.position.z = pose.getOrigin().getZ();
    currentArPose.orientation.x = pose.getRotation().getX();
    currentArPose.orientation.y = pose.getRotation().getY();
    currentArPose.orientation.z = pose.getRotation().getZ();
    currentArPose.orientation.w = pose.getRotation().getW();


}

 geometry_msgs::Pose VisionInterface::getArPose(){
     return currentArPose;

 }
