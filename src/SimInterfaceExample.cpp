#include <cstdio>
#include <iostream>
#include <fstream>


#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "robot/SimInterface.h"


using namespace std;


int t = 1.8 * 1e4;


int main(int argc, char** args) {

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    SimInterface* simI= new SimInterface (argc, args, t, *node);

       //adding ball with default values
        simI->addPrimShape(2,"ball");
        float nPos1[3] = {0.7, 0.8, 0.2};
        float nOr1[4]={0.0, 0.2, 0.0, 0.1};
        simI->setObjPose("ball",nPos1,nOr1);

        simI->addPrimShape(3,"cylinder");

        simI->setObjPose("cylinder",nPos1,nOr1);

        simI->addPrimShape(4,"my_cone1");

       //adding box with default values
       // simI->addPrimShape(1,"my_box");

       //importing from a file
       simI->importMesh("dish","/home/c7031098/iis_robot_sw/iis_catkin_ws/src/kukadu/src/objects/dish.stl");

       //checking the pose

       geometry_msgs::Pose Pose;
       simI->getObjPose("dish",Pose);
       cout<<Pose<< endl;

      //simI->getObjPose("my_box",Pose);
      // cout<<Pose<< endl;

       //adding box with specified dimensions and position

       float newP[3]={0,0,0.8};
       float newO[4]={0,0,0,0};
       float dim[3]= {0.2,0.2,0.3};
       simI->addPrimShape(1,"box1",newP,newO, dim,0.2);

       //setting the new pose for the object

       float nPos[3] = {0.5, 0.5, 0.5};
       float nOr[4]={0.0, 0.2, 0.0, 0.1};
       simI->setObjPose("dish",nPos,nOr);

       //deleting the objects from scene
      // simI->removeObj("dish");
       simI->removeObj("box1");


    return 0;

}


