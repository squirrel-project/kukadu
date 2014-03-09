#include <cstdio>
#include <iostream>
#include <fstream>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <queue>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_poly.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <vector>

#include "../include/kukadu.h"

using namespace std;

int main(int argc, char** args) {

    string rightHandPort = "/dev/ttyUSB0";
    string leftHandPort = "/dev/ttyUSB1";

    string left_prefix = "left_arm";
    string right_prefix = "right_arm";

    string deviceType = "real";

    string screwFile = "/home/shangl/newest.txt";

    int doRightOperation = 1;
    int doLeftOperation = 1;

    double handVelocity = 20.0;
    double az = 48.0;
    double bz = (az - 1) / 4;

    int kukaStepWaitTime = 1.8 * 1e4;

    float pendInitJoints[7] = {1.79281, -1.3077, -1.36712, -0.847933, -0.254344, 1.23292, 2.18119};
    float pendGraspJoints[7] = {2.83388, -1.81583, -1.2572, -0.556379, 0.0867394, 1.13701, 1.01905};
    float pendIntermedJoints[7] = {2.15394, -1.72848, -1.43927, -1.40072, -0.342002, 0.603178, 1.73683};
    float pendHoldJoints[7] = {2.79726, -1.45943, -1.4342, -0.53839, -0.561688, 1.47118, 2.27169};

    float headInitJoints[7] = {0.782031, 1.6012, 1.0254, 1.6749, 0.119783, -0.599311, 1.11514};
    float headGraspJoints[7] = {-0.127053, 1.03195, 1.1036, 1.0663, -0.276566, -0.886426, 1.43944};
    float headIntermedJoints[7] = {1.14941, 1.12815, 1.08387, 1.93618, -0.13507, -0.345952, 1.70287};

    // has to be replaced
    float headScrewJoints[7] = {1.14941, 1.12815, 1.08387, 1.93618, -0.13507, -0.345952, 1.70287};

    ros::NodeHandle* node = NULL;

    ControlQueue* leQueue = NULL;
    ControlQueue* raQueue = NULL;

    thread* raThr = NULL;

    ros::init(argc, args, "kukadu"); node = new ros::NodeHandle(); usleep(1e6);

    // execute screwing
    RosSchunk* raHand = NULL;

    if(doRightOperation)
        raHand = new RosSchunk(*node, "/real/right_sdh/follow_joint_trajectory/goal", "/real/right_sdh/joint_control/joint_states", "right");

    RosSchunk* leHand = NULL;

    if(doLeftOperation)
        leHand = new RosSchunk(*node, "/real/left_sdh/follow_joint_trajectory/goal", "/real/left_sdh/joint_control/joint_states", "left");

    if(doRightOperation)
        raHand->setGrasp(SDH::cSDHBase::eGID_CENTRICAL);

    if(doLeftOperation)
        leHand->setGrasp(SDH::cSDHBase::eGID_PARALLEL);

    // open hands initially
    if(doRightOperation)
        raHand->closeHand(0.1, handVelocity);

    if(doLeftOperation)
        leHand->closeHand(0.1, handVelocity);

    raQueue = new OrocosControlQueue(argc, args, kukaStepWaitTime,
                                     "/" + deviceType + "/" + right_prefix + "/joint_control/move",
                                     "/" + deviceType + "/" + right_prefix + "/joint_control/get_state",
                                     "/" + deviceType + "/" + right_prefix + "/settings/switch_mode",
                                     "/" + deviceType + "/" + right_prefix + "/cartesian_control/get_pose",
                                     "/" + deviceType + "/" + right_prefix + "/cartesian_control/set_impedance",
                                     "/" + deviceType + "/" + right_prefix + "/joint_control/set_impedance",
                                     "/" + deviceType + "/" + right_prefix + "/joint_control/ptp",
                                     "/" + deviceType + "/" + right_prefix + "/settings/get_command_state",
                                     "/" + deviceType + "/" + right_prefix + "/joint_control/ptp_reached",
                                     "not supported yet",
                                     *node);

    leQueue = new OrocosControlQueue(argc, args, kukaStepWaitTime,
                                     "/" + deviceType + "/" + left_prefix + "/joint_control/move",
                                     "/" + deviceType + "/" + left_prefix + "/joint_control/get_state",
                                     "/" + deviceType + "/" + left_prefix + "/settings/switch_mode",
                                     "/" + deviceType + "/" + left_prefix + "/cartesian_control/get_pose",
                                     "/" + deviceType + "/" + left_prefix + "/cartesian_control/set_impedance",
                                     "/" + deviceType + "/" + left_prefix + "/joint_control/set_impedance",
                                     "/" + deviceType + "/" + left_prefix + "/joint_control/ptp",
                                     "/" + deviceType + "/" + left_prefix + "/settings/get_command_state",
                                     "/" + deviceType + "/" + left_prefix + "/joint_control/ptp_reached",
                                     "not supported yet",
                                     *node);

    cout << "(main) connection to arms established" << endl;

    if(doRightOperation)
        raHand->connectHand();

    if(doLeftOperation)
        leHand->connectHand();

    if(doRightOperation) {
        raQueue->stopCurrentMode();
        raQueue->switchMode(10);
    }

    if(doLeftOperation) {
        leQueue->stopCurrentMode();
        leQueue->switchMode(10);
    }

    // move to initial position
    if(doRightOperation)
        raQueue->moveJoints(headInitJoints);

    if(doLeftOperation)
        leQueue->moveJoints(pendInitJoints);

    // go to grasp position
    if(doRightOperation)
        raQueue->moveJoints(headGraspJoints);

    if(doLeftOperation)
        leQueue->moveJoints(pendGraspJoints);

    // close hands
    if(doRightOperation)
        raHand->closeHand(1.0, handVelocity);

    if(doLeftOperation)
        leHand->closeHand(1.0, handVelocity);

    // pick up stuff
    if(doRightOperation)
        raQueue->moveJoints(headIntermedJoints);

    if(doLeftOperation)
        leQueue->moveJoints(pendIntermedJoints);

    // prepare for execution
    if(doRightOperation)
        raQueue->moveJoints(headScrewJoints);

    if(doLeftOperation)
        leQueue->moveJoints(pendHoldJoints);

    /*
    // ...start thread for screwing execution...
    raThr = raQueue->startQueueThread();

    // switch to impedance mode
    raQueue->switchMode(30);

    // execute screwing
    executeDemo(raQueue, screwFile.c_str(), doSimulation, az, bz, 1);

    // open hand after execution
    raHand->closeHand(0.0, handVelocity);

    // move arms away
    raQueue->moveJoints(headScrewJoints);
    leQueue->moveJoints(pendIntermedJoints);

    // open hands
    raHand->closeHand(0.0, handVelocity);
    leHand->closeHand(0.0, handVelocity);

    */

    // stopping all the execution
    if(doRightOperation)
        raQueue->stopCurrentMode();

    if(doLeftOperation)
        leQueue->stopCurrentMode();

//    raHand->disconnectHand();
//    leHand->disconnectHand();
	
    return 0;
    
}
