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

    int simulate = 0;

    string rightHandPort = "/dev/ttyUSB0";
    string leftHandPort = "/dev/ttyUSB1";

    string left_prefix = "left_arm";
    string right_prefix = "right_arm";

    string deviceType = "real";

    string screwFile = "/home/shangl/right_wot_screw.txt";

    int doRightOperation = 1;
    int doLeftOperation = 0;

    double handVelocity = 20.0;
    double az = 48.0;
    double bz = (az - 1) / 4;

    int kukaStepWaitTime = 1.8 * 1e4;

    double pendInitJointsTmp[7] = {1.79281, -1.3077, -1.36712, -0.847933, -0.254344, 1.23292, 2.18119};


    // with foam
    // float pendGraspJoints[7] = {2.83388, -1.81583, -1.2572, -0.556379, 0.0867394, 1.13701, 1.01905};

    // without foam
    double pendGraspJointsTmp[7] = {2.75685, -1.52887, -1.51049, -0.783576, 0.360113, 1.0022, 1.07122};


    double pendIntermedJointsTmp[7] = {2.15394, -1.72848, -1.43927, -1.40072, -0.342002, 0.603178, 1.73683};


    // with foam
    // float pendHoldJoints[7] = {2.70941, -1.12971, -1.87052, -0.796686, -0.460548, 1.02672, 2.47593};

    // without foam
    double pendHoldJointsTmp[7] = {2.883692741394043, -1.4105281829833984, -1.712611198425293, -0.6825535297393799, -1.0014642477035522, 0.8458623886108398, 2.6116816997528076};


    double headInitJointsTmp[7] = {0.864292, 1.54993, 0.872115, 1.77397, -2.81503, 0.464614, -1.73203};


    // with foam
    // float headGraspJoints[7] = {-0.127053, 1.03195, 1.1036, 1.0663, -0.276566, -0.886426, 1.43944};

    // without foam
    double headGraspJointsTmp[7] = {-0.639902, 0.901709, 0.641415, 0.382029, -2.51635, 1.13727, -1.72087};


    double headIntermedJointsTmp[7] = {0.71328, 1.25731, 1.14691, 2.00651, -2.65152, 0.192659, -1.92759};


    arma::vec pendInitJoints = createArmaVecFromDoubleArray(pendInitJointsTmp, 7);
    arma::vec pendGraspJoints = createArmaVecFromDoubleArray(pendGraspJointsTmp, 7);
    arma::vec pendIntermedJoints = createArmaVecFromDoubleArray(pendIntermedJointsTmp, 7);
    arma::vec pendHoldJoints = createArmaVecFromDoubleArray(pendHoldJointsTmp, 7);
    arma::vec headInitJoints = createArmaVecFromDoubleArray(headInitJointsTmp, 7);
    arma::vec headGraspJoints = createArmaVecFromDoubleArray(headGraspJointsTmp, 7);
    arma::vec headIntermedJoints = createArmaVecFromDoubleArray(headIntermedJointsTmp, 7);

    // has to be replaced
//    float headScrewJoints[7] = {0.16576188802719116, 1.3929321765899658, 1.319714069366455, 1.9400454759597778, -2.738997459411621, -0.0808953121304512, -1.830229640007019};

    // with foam
    // float screwStart[7] = {0.287568, 1.66721, 1.46937, 2.08513, -2.8281, -0.0255578, -1.7752};

    // without foam
    double screwStartTmp[7] = {0.3357972502708435, 1.236600399017334, 1.2974035739898682, 2.049152374267578, -2.7408792972564697, -0.10395102202892303, -1.8124815225601196};
    arma::vec screwStart = createArmaVecFromDoubleArray(screwStartTmp, 7);

    ros::NodeHandle* node = NULL;

    std::shared_ptr<ControlQueue> leQueue = std::shared_ptr<ControlQueue>(nullptr);
    std::shared_ptr<ControlQueue> raQueue = std::shared_ptr<ControlQueue>(nullptr);

    thread* raThr = NULL;

    ros::init(argc, args, "kukadu"); node = new ros::NodeHandle(); usleep(1e6);

    // execute screwing
    RosSchunk* raHand = NULL;
    RosSchunk* leHand = NULL;

    if(!simulate) {

        if(doRightOperation)
            raHand = new RosSchunk(*node, "/real/right_sdh/follow_joint_trajectory/goal", "/real/right_sdh/joint_control/joint_states", "right");

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

        if(doRightOperation)
            raQueue = std::shared_ptr<ControlQueue>(new OrocosControlQueue(argc, args, kukaStepWaitTime,
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
                                         *node));

        if(doLeftOperation)
            leQueue = std::shared_ptr<ControlQueue>(new OrocosControlQueue(argc, args, kukaStepWaitTime,
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
                                         *node));

        cout << "(main) connection to arms established" << endl;

        if(doRightOperation)
            raHand->connectHand();

        if(doLeftOperation)
            leHand->connectHand();

        if(doRightOperation) {
            raQueue->stopCurrentMode();
        //    raQueue->setStiffness(2000, 150, 0.7, 7.0, 70, 2.0);
        //    raQueue->setStiffness(4000, 250, 0.7, 7.0, 70, 2.0);
        //    raQueue->switchMode(30);
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

        if(doRightOperation)
            raQueue->moveJoints(headGraspJoints);

        if(doLeftOperation)
            leQueue->moveJoints(pendGraspJoints);

        // go to grasp position
        if(doRightOperation) {
            raHand->closeHand(1.0, handVelocity);
            raQueue->moveJoints(headIntermedJoints);
        }


        if(doLeftOperation) {
            leHand->closeHand(1.0, handVelocity);
            leQueue->moveJoints(pendIntermedJoints);
        }

        // prepare for execution
//        if(doRightOperation)
//            raQueue->moveJoints(headScrewJoints);

        if(doLeftOperation)
            leQueue->moveJoints(pendHoldJoints);

        // get right arm ready for execution
        if(doRightOperation)
            raQueue->moveJoints(screwStart);

        // ...start thread for screwing execution...
        if(doRightOperation)
            raThr = raQueue->startQueueThread();

        // switch to impedance mode
//        raQueue->switchMode(30);


    }

    // execute screwing
    if(doRightOperation) {
        executeDemo(raQueue, screwFile.c_str(), simulate, az, bz, 1);
        cout << "finished demo execution" << endl;
    }

    if(!simulate) {

        // open hand after execution
        raHand->closeHand(0.0, handVelocity);
//        leHand->closeHand(0.0, handVelocity);

        // stopping all the execution
        if(doRightOperation)
            raQueue->stopCurrentMode();

        if(doLeftOperation)
            leQueue->stopCurrentMode();

//        raHand->disconnectHand();
//        leHand->disconnectHand();

    }

    return 0;

}
