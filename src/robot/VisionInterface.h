#ifndef VISIONINTERFACE_H
#define VISIONINTERFACE_H

#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/Pose.h"
#include "Eigen/Dense"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <tf/tfMessage.h>

#include "../utils/utils.h"


class VisionInterface {
private:
    int argc;
    int sleepTime;

    char** argv;

    ros::NodeHandle node;

    std::string currentCameraTag;

    std::string arTagTopic;

    ros::Subscriber subArTag;

    bool arTagTracker;
    bool firstSet;

    geometry_msgs::Pose currentArPose;
    tf::Transform tfChestKin;

public:
    VisionInterface(int argc, char** argv, int sleepTime, ros::NodeHandle node);
    VisionInterface(int argc, char** argv, int sleepTime, std::string cameraTag, ros::NodeHandle node);

    void construct();

    void setCameraTag(std::string cameraTag);

    void setArTagTracker();

    void arTagCallback(const tf::tfMessage& msg);

    geometry_msgs::Pose getArPose();


};

#endif // VISIONINTERFACE_H
