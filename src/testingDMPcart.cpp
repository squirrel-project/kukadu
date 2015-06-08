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
#include "utils/utils.h"


using namespace std;
using namespace arma;

int test=1; //testing trajectory without dmps
//int test=2; //testing trajectory with dmps


std::string hand="left";
std::string arm="left_arm";
std::string environment="simulation";

//string hand="right";
//string arm="right_arm";


std::string resolvePath(std::string path);
geometry_msgs::Pose vectordouble2pose(std::vector<double>* vectorpose);
arma::mat readMovements(std::string file);

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

    //if (environment=="simulation"){
    thread* inputThr = NULL;
    SimInterface* simI= new SimInterface (argc, args, kukaStepWaitTime, *node);
    cout<<"sim interface created"<<endl;

    RosSchunk* handQ=new RosSchunk(*node, environment, hand);
    cout<<"hand interface created"<<endl;

    OrocosControlQueue* queue = new OrocosControlQueue(kukaStepWaitTime, environment, arm, *node);
    cout<<"control queue interface created"<<endl;


    //moving hand to staring position
    vector<double> newPos= {0, -1.57, 0, -1.57,0,-1.57,0};

    handQ->publishSdhJoints(newPos);
    queue->stopCurrentMode();
    queue->switchMode(10);
    queue->startQueueThread();
    cout<<"queue started"<<endl;

    vector<double> PoC={0.272241000000000,	0.876041000000000,	0.315563000000000,	0.776795000000000,	0.0112587000000000,	-0.0194455000000000,	0.629462000000000};
    // float newPoC[7]={0.299991,	0.920158,	0.397039,	0.776795,	0.0112587,	-0.0194455,	0.629462233312};
    //queue->moveCartQuats(newPoC);
   // vector<double> newPoC1={0.272241000000000,	0.876041000000000,	0.315563000000000,	0.776795000000000,	0.0112587000000000,	-0.0194455000000000,	0.629462000000000};
   // vector<double> newPoC1={0.272241000000000,	0.876041000000000,	0.395563000000000,	0.776795000000000,	0.0112587000000000,	-0.0194455000000000,	0.629462000000000};
    vector<double> newPoC1={0.0,	0.0,	0.0,	0.776795000000000,	0.0112587000000000,	-0.0194455000000000,	0.629462000000000};

    arma::vec Rpos={0.299991,	0.920158,	0.397039,	0.776795,	0.0112587,	-0.0194455,	6.86636e-44};
    //queue->moveCartesian(vectordouble2pose(&Rpos));

    cout<<"importing objects"<<endl;

    float newP[3]={0.5,0.7,0.01};
    float newO[4]={0,0,0,0};
    float dim[3]= {1,2,0.08};

    simI->addPrimShape(1,"sponge",newP,newO, dim,1);
    simI->setObjMaterial("sponge","lowFrictionMaterial");

    sleep(1);

   // arma::vec startPosLeft={-0.7234392762184143, 0.9734601378440857, 1.2643691301345825, -0.9762417674064636, -0.5300372242927551, 1.1202027797698975, 0.3392448425292969};
   // queue->moveJoints(Rpos);
    //float Rpos[7]={0.343906,	0.381331,	0.384666,	0.00919376,	0.679607,	0.729507,	0.0765885};
      //queue->moveCartQuats(Rpos);

    switch(test){

    case 1:{



        sleep(2);

        cout <<"reproducing movement without dmps"<<endl;

        float newP1[3]={0.30,0.7,0.2};
        //float newP1[3]={0.23,0.7,0.2};
        //float newP1[3]={0.25,0.7,0.2};
        float newO1[4]={0,0,0,0};
        float dim1[3]= {0.2,0.2,0.1};

        // float newP1[3]={0.27,0.7,0.2}; //first
        string path="/home/c7031098/Documents/Work/pushing/models/2box.stl";
      //  simI->importMesh("2box",path,newP1, newO1, 0.0, 0.8);
       // simI->setObjMaterial("2box","lowFrictionMaterial");
        sleep(2);
        cout<<"moving to starting position"<<endl;
        queue->moveCartesian(vectordouble2pose(&newPoC1));
        sleep(2);
        cout<<"executing trajectory"<<endl;

        //executeTrajCart(queue, resolvePath("/home/c7031098/testing/Rcart1.txt"));
      //  executeTrajCart(queue,resolvePath("/home/c7031098/testing/dataCart/car22.txt"));

        break;
    }

    case 2:{


        float newP1[3]={0.30,0.7,0.2};
        //float newP1[3]={0.23,0.7,0.2};
        //float newP1[3]={0.25,0.7,0.2};
        float newO1[4]={0,0,0,0};
        float dim1[3]= {0.2,0.2,0.1};
         string path="/home/c7031098/Documents/Work/pushing/models/2box.stl";

        // float newP1[3]={0.27,0.7,0.2}; //first
        simI->importMesh("2box",path,newP1, newO1, 0.0, 0.8);
        simI->setObjMaterial("2box","lowFrictionMaterial");
        sleep(5);
        queue->moveCartesian(vectordouble2pose(&newPoC1));

        cout<<"in starting position C"<<endl;
        sleep(5);
       //  t_executor_res demoRes = executeDMPcart(queue, resolvePath("/home/c7031098/testing/dataCart/car12.txt"), resolvePath("/home/c7031098/testing/dataCart/o25d1.txt"),  0, az, bz, 0,simI, "2box");


        break;
    }


    }




    /*
sleep(2);


executeTrajCart(queue, resolvePath("/home/c7031098/testing/Rcart1.txt"));

switch(experiment){



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


geometry_msgs::Pose vectordouble2pose(std::vector<double>* vectorpose){

    geometry_msgs:: Pose posepose;
    posepose.position.x=vectorpose->at(0);
    posepose.position.y=vectorpose->at(1);
    posepose.position.z=vectorpose->at(2);
    posepose.orientation.x=vectorpose->at(3);
    posepose.orientation.y=vectorpose->at(4);
    posepose.orientation.z=vectorpose->at(5);
    posepose.orientation.w=vectorpose->at(6);

    return posepose;

}

void executeTrajCart(OrocosControlQueue* movementQu, string file){



    int columns = 8;
    mat coord = readMovements(file);
    int duration=coord.n_rows;
    movementQu->stopCurrentMode();
    movementQu->switchMode(20);
    cout<<"starting movement"<<endl;
    sleep(0.5);


    for (int i =0; i < duration; i=i+5) {
        vector<double> moveCarts;
        for(int j = 0; j < 7; ++j){
            sleep(0.5);
            cout<<coord(i,j+1)<<" ";
            moveCarts.push_back(coord(i,j+1));

        }
        cout<<endl;

       // movementQu->addCartesianPosToQueue(vectordouble2pose(&moveCarts));
        vector<double> diff={0.0,	0.00,	0.08,	0.0,	0.0,	0.0,	1.0};
        movementQu->moveCartesianRelativeWf(vectordouble2pose(&moveCarts), vectordouble2pose(&diff));



        // movementQu->moveCartesian(vectordouble2pose(&moveCarts));

    }

}

