#include "Kinect.hpp"

using namespace std;

Kinect::Kinect(ros::NodeHandle node) {
    string kinectPrefix = "kinect";
    construct(kinectPrefix, kinectPrefix + "_depth_frame", node);
}

Kinect::Kinect(std::string kinectPrefix, ros::NodeHandle node) {
    construct(kinectPrefix, kinectPrefix + "_depth_frame", node);
}

Kinect::Kinect(std::string kinectPrefix, std::string targetFrame, ros::NodeHandle node) {
    construct(kinectPrefix, targetFrame, node);
}

void Kinect::construct(std::string kinectPrefix, std::string targetFrame, ros::NodeHandle node) {

    isInit = false;
    keepRunning = false;
    pcRequested = false;
    firstCloudSet = false;

    this->node = node;

    this->kinectPrefix = "/" + kinectPrefix;
    subKinect = node.subscribe(kinectPrefix + "/depth_registered/points", 1, &Kinect::callbackKinectPointCloud, this);

    transformListener = shared_ptr<tf::TransformListener>(new tf::TransformListener());

    sleep(1);

    this->targetFrame = targetFrame;

}

std::shared_ptr<std::thread> Kinect::startSensing() {
    keepRunning = true;
    thr = std::shared_ptr<std::thread>(new std::thread(&Kinect::runThread, this));
    while(!this->isInitialized());
    return thr;
}

void Kinect::stopSensing() {
    keepRunning = false;
}

void Kinect::runThread() {

    isInit = true;
    ros::Rate r(10);
    while(keepRunning) {
        ros::spinOnce();
        r.sleep();
    }

}

bool Kinect::isInitialized() {
    return isInit;
}

void Kinect::callbackKinectPointCloud(const sensor_msgs::PointCloud2& pc) {

    pcMutex.lock();

        if(pcRequested) {
            currentPc = pc;
        }
        pcRequested = false;
        firstCloudSet = true;

    pcMutex.unlock();

}

sensor_msgs::PointCloud2 Kinect::getCurrentPointCloud() {

    pcRequested = true;
    ros::Rate r(10);
    while(!firstCloudSet)
        r.sleep();

    pcMutex.lock();

        sensor_msgs::PointCloud2 retCloud = currentPc;
        firstCloudSet = false;

    pcMutex.unlock();

    pcl_ros::transformPointCloud(targetFrame, retCloud, retCloud, *transformListener);

    return retCloud;

}
