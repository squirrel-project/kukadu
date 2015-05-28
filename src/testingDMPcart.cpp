#include <cstdio>
#include <iostream>
#include <fstream>
#include <thread>
#include <string>
#include <vector>
#include <wordexp.h>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "../include/kukadu.h"

#define DOSIMULATION 0

using namespace std;


int experiment=1;
int adapt=0;


std::string hand="left";
std::string arm="left_arm";
std::string environment="simulation";

//string hand="right_sdh";
//string arm="right_arm";


std::string resolvePath(std::string path);

void executeTrajCart(OrocosControlQueue* movementQu, string file);
void collectDataCart(OrocosControlQueue* queue, SimInterface* sim, string object_id);
t_executor_res executeDMPcart(OrocosControlQueue* movementQu, string file, string file1, int doSimulation, double az, double bz, int plotResults, SimInterface* simI,string object_id);
t_executor_res executePushCart(OrocosControlQueue* movementQu, string file, string file1, int doSimulation, double az, double bz, int plotResults, SimInterface* simI,string object_id);

double az = 48.0;
double bz = (az - 1) / 4;

int temp=0;

SimInterface *sim;
int kukaStepWaitTime = 1.8 * 1e4;
double dmpStepSize = kukaStepWaitTime * 1e-6;
//void collectData(OrocosControlQueueExt* queue);



std::string resolvePath(std::string path) {

    wordexp_t p;
    wordexp(path.c_str(), &p, 0 );
    char** w = p.we_wordv;
    string ret = string(*w);
    wordfree( &p );

    return ret;

}

int main(int argc, char** args) {

  cout<<"test start; creating interfaces"<<endl;

  ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);

  if (environment=="simulation"){
  thread* inputThr = NULL;
  SimInterface* simI= new SimInterface (argc, args, kukaStepWaitTime, *node);
  cout<<"sim interface created"<<endl;}

  RosSchunk* handQ=new RosSchunk(*node, environment, hand);
  cout<<"hand interface created"<<endl;

  OrocosControlQueue* queue = new OrocosControlQueue(kukaStepWaitTime, environment, arm, *node);
  cout<<"control queue interface created"<<endl;


  //moving hand to staring position
  vector<double> newPos= {0, -1.57, 0, -1.57,0,-1.57,0};

  handQ->publishSdhJoints(newPos);

/*
  cout <<"version experiment 7"<<endl;

  queue->stopCurrentMode();
  queue->switchMode(10);
  queue->setRunMode(1); //setting to cartesian mode
 // queue->setVelocityPtp(0.5);
 // queue->startQueueThread();
  cout<<"queue started"<<endl;
  float newPoC1[7]={0.272241000000000,	0.876041000000000,	0.315563000000000,	0.776795000000000,	0.0112587000000000,	-0.0194455000000000,	0.629462000000000};
 // float newPoC[7]={0.299991,	0.920158,	0.397039,	0.776795,	0.0112587,	-0.0194455,	0.629462233312};
  //queue->moveCartQuats(newPoC);
//  float newPoC1[7]={0.272241000000000,	0.876041000000000,	0.315563000000000,	0.776795000000000,	0.0112587000000000,	-0.0194455000000000,	0.629462000000000};
  float Rpos[7]={0.343906,	0.381331,	0.384666,	0.00919376,	0.679607,	0.729507,	0.0765885};
  //queue->moveCartQuats(Rpos);

  sleep(5);
  cout<<"in starting position "<<endl;



  float newP[3]={0.5,0.7,0.01};
   float newO[4]={0,0,0,0};
   float dim[3]= {1,2,0.08};

 simI->addPrimShape(1,"sponge",newP,newO, dim,1);
 simI->setObjMaterial("sponge","lowFrictionMaterial");


sleep(2);


executeTrajCart(queue, resolvePath("/home/c7031098/testing/Rcart1.txt"));

switch(experiment){

case 1:{

 string path="/home/c7031098/Documents/Work/pushing/models/2box.stl";

 float newP1[3]={0.30,0.7,0.2};
 //float newP1[3]={0.23,0.7,0.2};
// float newP1[3]={0.25,0.7,0.2};
 float newO1[4]={0,0,0,0};
 float dim1[3]= {0.2,0.2,0.1};

// float newP1[3]={0.27,0.7,0.2}; //first
 simI->importMesh("2box",path,newP1, newO1, 0.0, 0.8);
 simI->setObjMaterial("2box","lowFrictionMaterial");
 sleep(5);
 queue->moveCartQuats(newPoC1);

 cout<<"in starting position C"<<endl;
  sleep(5);
  t_executor_res demoRes = executeDMPcart(queue, resolvePath("/home/c7031098/testing/dataCart/car12.txt"), resolvePath("/home/c7031098/testing/dataCart/o25d1.txt"),  0, az, bz, 0,simI, "2box");
break;
}

case 2: {
    float newP1[3]={0.23,0.7,0.1};
    float newO1[4]={0,0,0,0};
    float dim1[3]= {0.2,0.2,0.1};
    simI->addPrimShape(1,"box1",newP1,newO1, dim1,0.4);
   sleep(1);
   simI->setObjMaterial("box1","lowFrictionMaterial");
    sleep(5);
    queue->moveCartQuats(newPoC1);

    cout<<"in starting position C"<<endl;
     sleep(5);
     t_executor_res demoRes = executeDMPcart(queue, resolvePath("/home/c7031098/testing/dataCart/car24.txt"), resolvePath("/home/c7031098/testing/dataCart/ob24.txt"),  0, az, bz, 0,simI, "box1");

break;
}

case 3: {
    simI->setObjMaterial("sponge","lowFrictionMaterial");
         float newP2[3]={0.25,0.7,0.2};
         float newO2[4]={0,0,0,0};
         float dim2[3]= {0.2,0.06};
         simI->addPrimShape(3,"bottle",newP2,newO2, dim2,0.4);
         sleep(3);

         simI->setObjPose("bottle",newP2,newO2);
        // inputThr = new thread(collectData,queue);
         simI->setObjMaterial("bottle","lowFrictionMaterial");
       sleep(3);
    queue->moveCartQuats(newPoC1);

    cout<<"in starting position C"<<endl;
     sleep(5);
     t_executor_res demoRes = executeDMPcart(queue, resolvePath("/home/c7031098/testing/dataCart/car31.txt"), resolvePath("/home/c7031098/testing/dataCart/ob3.txt"),  0, az, bz, 0,simI, "bottle");

break;
}
case 5: {
    simI->setObjMaterial("sponge","lowFrictionMaterial");
         float newP2[3]={0.25,0.7,0.2};
         float newO2[4]={0,0,0,0};
         float dim2[3]= {0.2,0.06};
         simI->addPrimShape(3,"bottle",newP2,newO2, dim2,0.4);
         sleep(3);

         simI->setObjPose("bottle",newP2,newO2);
        // inputThr = new thread(collectData,queue);
         simI->setObjMaterial("bottle","lowFrictionMaterial");
       sleep(3);
   // queue->moveCartQuats(newPoC1);

    cout<<"in starting position C"<<endl;
     sleep(5);
    t_executor_res demoRes = executeDMPcart(queue, resolvePath("/home/c7031098/testing/Rcart1.txt"), resolvePath("/home/c7031098/testing/dataCart/ob3.txt"),  0, az, bz, 0,simI, "bottle");

break;
}

case 6: {
    simI->setObjMaterial("sponge","lowFrictionMaterial");
         float newP2[3]={0.25,0.7,0.2};
         float newO2[4]={0,0,0,0};
          float dim2[3]= {0.12};
        // simI->addPrimShape(3,"bottle",newP2,newO2, dim2,0.4);
       /*  simI->addPrimShape(2,"ball",newP2,newO2, dim2,1);
         simI->setObjMaterial("ball","usr_rubber1");
         sleep(3);

         simI->setObjPose("ball",newP2,newO2); */
        // inputThr = new thread(collectData,queue);
       //  simI->setObjMaterial("bottle","lowFrictionMaterial");
    /*      string path="/home/c7031098/Documents/Work/pushing/models/2box.stl";

          float newP1[3]={0.30,0.75,0.2};
          //float newP1[3]={0.23,0.7,0.2};
         // float newP1[3]={0.25,0.7,0.2};
          float newO1[4]={0,0,0,0};
          float dim1[3]= {0.2,0.2,0.1};

         // float newP1[3]={0.27,0.7,0.2}; //first
          simI->importMesh("2box",path,newP1, newO1, 0.0, 0.2);
          simI->setObjMaterial("2box","lowFrictionMaterial");
       sleep(3);
   // queue->moveCartQuats(newPoC1);

    cout<<"in starting position C"<<endl;
     sleep(5);
  t_executor_res demoRes = executeDMPcart(queue, resolvePath("/home/c7031098/testing/dataCart/gen2.txt"), resolvePath("/home/c7031098/testing/dataCart/ogen1.txt"),  0, az, bz, 0,simI, "2box");
   //  t_executor_res demoRes = executeDMPcart(queue, resolvePath("/home/c7031098/testing/push2c.txt"), resolvePath("/home/c7031098/testing/push2w.txt"),  0, az, bz, 0,simI, "2box");

break;
}
case 7: {
    cout<<"version new"<<endl<<endl<<endl<<endl;

    float newPoC[7]={-1.0887566804885864, 0.769572377204895, 1.6097323894500732, -1.1658289432525635, -0.46473878622055054, 1.042203426361084, 0.6838330626487732};

   //S queue->moveJoints(newPoC);

   // cout<<"in starting position C"<<endl;

     sleep(5);
     cout<<"testing dmp"<< endl;
     //St_executor_res demoRes = executeDMPcart(queue, resolvePath("/home/c7031098/testing/push2c.txt"), resolvePath("/home/c7031098/testing/push2w.txt"),  0, az, bz, 0,simI, "2box");
     cout<<"testing traject"<< endl;
     executeTrajCart(queue, resolvePath("/home/c7031098/testing/push2c.txt"));
break;
}
}*/
 //t_executor_res demoRes = executeDMPcart(queue, resolvePath("/home/c7031098/testing/dataCart/car12.txt"), resolvePath("/home/c7031098/testing/dataCart/obd1.txt"),  0, az, bz, 0,simI, "2box");
 //inputThr = new thread(collectDataCart,queue,simI,"box1");
 //cout<<"collecting data started"<<endl;
 //executeTrajCart(queue, resolvePath("/home/c7031098/testing/data1.txt"));
}

