#ifndef KINECT_H
#define KINECT_H

#include <cstdio>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <wordexp.h>
#include <memory>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/PointCloud2.h>

class Kinect {

private:

    bool isInit;
    bool keepRunning;
    bool firstCloudSet;

    bool pcRequested;

    std::shared_ptr<std::thread> thr;

    std::mutex pcMutex;

    std::string targetFrame;
    std::string kinectPrefix;

    std::shared_ptr<tf::TransformListener> transformListener;

    sensor_msgs::PointCloud2 currentPc;

    ros::NodeHandle node;

    ros::Subscriber subKinect;
    ros::Subscriber subTransformation;

    void runThread();
    void callbackKinectPointCloud(const sensor_msgs::PointCloud2& pc);
    void construct(std::string kinectPrefix, std::string targetFrame, ros::NodeHandle node);

public:

    Kinect(ros::NodeHandle node);
    Kinect(std::string kinectPrefix, ros::NodeHandle node);
    Kinect(std::string kinectPrefix, std::string targetFrame, ros::NodeHandle node);

    void stopSensing();
    std::shared_ptr<std::thread> startSensing();

    bool isInitialized();

    sensor_msgs::PointCloud2 getCurrentPointCloud();

};



#endif // KINECT_H
