
#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>

#include "../include/kukadu.h"

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>



using namespace std;
using namespace arma;

int kukaStepWaitTime = 14 * 1e4;
float pickMaxAxisTorque = 2.0;
double az = 48.0;
double bz = (az - 1) / 4;

std::string hand = "left";
std::string arm = "left_arm";
std::string environment = "simulation";

int main(int argc, char** args) {

    ros::init(argc, args, "push");

    ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);

    std::shared_ptr<KukieControlQueue> queue = std::shared_ptr<KukieControlQueue>(new KukieControlQueue(kukaStepWaitTime, environment, arm, *node));

    queue = std::shared_ptr<KukieControlQueue>(new KukieControlQueue(kukaStepWaitTime, environment, arm, *node));

    RosSchunk* handQ=new RosSchunk(*node, environment, hand);
    cout<<"hand interface created"<<endl;

    //moving hand to staring position
    vector<double> newPos = {0, -1.57, 0, -1.57, 0, -1.57, 0};
    handQ->publishSdhJoints(newPos);


    cout<< "initilization done"<<endl;


    shared_ptr<SensorData> data = SensorStorage::readStorage(queue, "/home/c7031098/testing/push_data/pushing_data/kuka_lwr_real_left_arm_0");
    data->removeDuplicateTimes();

    arma::vec times = data->getTimes();
    arma::mat jointPos = data->getJointPos();


    queue->stopCurrentMode();
    queue->switchMode(10);

    vector<double> tmpmys{0, 1, 2, 3, 4, 4.1, 4.3, 4.5};
    //vector<double> tmpmys;
    vector<double> tmpsigmas{0.2, 0.8};


    JointDMPLearner learner(az, bz, join_rows(times, jointPos));
    std::shared_ptr<Dmp> Ldmp = learner.fitTrajectories();

    cout<< "moving to start position"<<endl;
    queue->moveJoints(Ldmp->getY0());

    sleep(5);

    double tStart = 0.0;
    double tEnd = 7.5;
    double tau = 0.8;


    DMPExecutor executorDmp(Ldmp, queue);
    executorDmp.simulateTrajectory();

    /*

    string file = "/home/c7031098/testing/testCar1/car1.txt";

    vector<double> tmpmys{0, 1, 2, 3, 4, 4.1, 4.3, 4.5};

    vector<double> tmpsigmas{0.2, 0.8};

    // reading in file

    cout << "reading file" << endl;
    mat carts = readMovements(file);
    tmpmys = constructDmpMys(carts);

    double tau = 0.8;
    double ax = -log((float)0.1) / carts(carts.n_rows - 1, 0) / tau;

    cout << "creating base" << endl;

    vector<DMPBase> baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);

    CartesianDMPLearner dmpLearner(baseDef, tau, az, bz, ax, carts);
    dmpLearner.fitTrajectories();
    //arma::vec startP=learnedDmps.getY0();*/



    return 0;
}


tf::Quaternion exp(const double* logQuat) {

    double x, y, z, w;
    double modR = sqrt(logQuat[0] * logQuat[0] + logQuat[1] * logQuat[1] + logQuat[2] * logQuat[2]);

    if (modR > 0) {

        w = cos(modR);
        x = sin(modR) * logQuat[0] / modR;
        y = sin(modR) * logQuat[1] / modR;
        z = sin(modR) * logQuat[2] / modR;

    } else {

        w = 0.0;
        x = 0.0;
        y = 0.0;
        z = 0.0;

    }

    return tf::Quaternion(x, y, z, w);

}





/*geometry_msgs::Vector3 log(const geometry_msgs::Quaternion quat) {

    geometry_msgs::Vector3 logQuat;
    double modU = sqrt(quat.x*quat.x  + quat.y*quat.y + quat.z*quat.z);

    if (modU > 0) {

        logQuat.x = quat.x/modU;
        logQuat.y = quat.y/modU;
        logQuat.z = quat.z/modU;

    } else {

        logQuat.x = 0.0;
        logQuat.y = 0.0;
        logQuat.z = 0.0;

    }

    return logQuat;

}




geometry_msgs::Quaternion exp(geometry_msgs::Vector3 logQuat) {

    geometry_msgs::Quaternion quat;
    double modR = sqrt(logQuat.x*logQuat.x + logQuat.y*logQuat.y + logQuat.z*logQuat.z);

    if (modR > 0) {

        quat.w = cos(modR);
        quat.x = sin(modR)*logQuat.x/modR;
        quat.y = sin(modR)*logQuat.y/modR;
        quat.z = sin(modR)*logQuat.z/modR;

    } else {

        quat.w = 0.0;
        quat.x = 0.0;
        quat.y = 0.0;
        quat.z = 0.0;

    }

    return quat;

}*/


