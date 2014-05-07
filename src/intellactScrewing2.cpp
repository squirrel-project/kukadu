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
#include <unistd.h>

#include "../include/kukadu.h"

using namespace std;

int main(int argc, char** args) {

    int simulate = 0;

    string rightHandPort = "/dev/ttyUSB0";
    string leftHandPort = "/dev/ttyUSB1";

    string left_prefix = "left_arm";
    string right_prefix = "right_arm";

    string deviceType = "real";

    string screwFile = "src/kukadu/movements/screwing_demo/mounted_screwing3.txt";

    // these two are switched in this version
    int doRightOperation = 1;
    int doLeftOperation = 0;

    int useHands = 1;

    double handVelocity = 20.0;
    double az = 48.0;
    double bz = (az - 1) / 4;

    int kukaStepWaitTime = 1.8 * 1e4;

    /*
    float pendInitJoints[7] = {2.489938974380493, -0.39937564730644226, -0.07285512238740921, 1.402941346168518, -2.369999408721924, 1.1789064407348633, 1.1796085834503174};
    float pendGraspJoints[7] = {2.9670770168304443, -1.0801817178726196, 0.7259281873703003, 0.7945775389671326, -2.61958646774292, 1.4365915060043335, 0.46473127603530884};
    float pendIntermedJoints[7] = {2.9670770168304443, -1.0801817178726196, 0.7259281873703003, 0.7945775389671326, -2.61958646774292, 1.4365915060043335, 0.46473127603530884};
    float pendHoldJoints[7] = {2.9670770168304443, -1.0801817178726196, 0.7259281873703003, 0.7945775389671326, -2.61958646774292, 1.4365915060043335, 0.46473127603530884};

    float headInitJoints[7] = {0.864292, 1.54993, 0.872115, 1.77397, -2.81503, 0.464614, -1.73203};
    float headGraspJoints[7] = {-0.10682, 1.0857, 1.01803, 1.15677, -0.234306, -0.802618, -1.57839};
    float headIntermedJoints[7] = {0.3357972502708435, 1.236600399017334, 1.2974035739898682, 2.049152374267578, -2.7408792972564697, -0.10395102202892303, -1.8124815225601196};
    float screwStart[7] = {0.275761, 1.94407, 1.66344, 2.04083, -2.79594, 0.0129723, -1.81238};
    */

    float pendInitJoints[7] = {-0.8447425961494446, 0.8997548818588257, 2.2580084800720215, 1.0912582874298096, 1.0713993310928345, -0.32208988070487976, 1.3560845851898193};
    float pendGraspJoints[7] = {-0.8447425961494446, 0.8997548818588257, 2.2580084800720215, 1.0912582874298096, 1.0713993310928345, -0.32208988070487976, 1.3560845851898193};
    float pendIntermedJoints[7] = {-0.8447425961494446, 0.8997548818588257, 2.2580084800720215, 1.0912582874298096, 1.0713993310928345, -0.32208988070487976, 1.3560845851898193};
//    float pendHoldJoints[7] = {-0.920188844203949, 0.6854411959648132, 2.3923866748809814, 1.3453123569488525, 1.7259241342544556, -0.7008364796638489, 0.47811609506607056};
    float pendHoldJoints[7] = {-0.8447425961494446, 0.8997548818588257, 2.2580084800720215, 1.0912582874298096, 1.0713993310928345, -0.32208988070487976, 1.3560845851898193};

    float headInitJoints[7] = {1.3073233366012573, -1.5534619092941284, 1.7766101360321045, 1.8323383331298828, -2.5772712230682373, 0.1932985484600067, -2.5928666591644287};
    float headGraspJoints[7] = {2.2317893505096436, -1.2508511543273926, 1.9576542377471924, 1.273720145225525, -2.7874672412872314, 0.7773531079292297, -2.590383291244507};
    float headIntermedJoints[7] = {1.8988120555877686, -1.292402744293213, 1.90839684009552, 1.3448892831802368, -2.78735089302063, 0.7773957848548889, -2.590383291244507};
    float screwStart[7] = {1.9282081127166748, -1.5321063995361328, 1.5018665790557861, 1.9316072463989258, -2.914626121520996, 0.033447593450546265, -2.592909336090088};

    char cCurrentPath[FILENAME_MAX];
    getcwd(cCurrentPath, sizeof(cCurrentPath));
    printf ("The current working directory is %s", cCurrentPath);

    ros::NodeHandle* node = NULL;

    ControlQueue* leQueue = NULL;
    ControlQueue* raQueue = NULL;

    thread* raThr = NULL;

    ros::init(argc, args, "kukadu"); node = new ros::NodeHandle(); usleep(1e6);

    // execute screwing
    RosSchunk* raHand = NULL;
    RosSchunk* leHand = NULL;

    if(!simulate) {

//        if(doRightOperation)
//            raHand = new RosSchunk(*node, "/real/right_sdh/follow_joint_trajectory/goal", "/real/right_sdh/joint_control/joint_states", "right");

        if(doRightOperation && useHands)
            leHand = new RosSchunk(*node, "/real/left_sdh/joint_control/move", "/real/left_sdh/joint_control/get_state", "left");

        RosSchunk* tmp;
        tmp = raHand;
        raHand = leHand;
        leHand = tmp;

        if(doRightOperation && useHands)
            raHand->setGrasp(SDH::cSDHBase::eGID_CENTRICAL);

//        if(doLeftOperation)
//            leHand->setGrasp(SDH::cSDHBase::eGID_PARALLEL);

        // open hands initially
        if(doRightOperation && useHands)
            raHand->closeHand(0.1, handVelocity);

//        if(doLeftOperation)
//            leHand->closeHand(0.1, handVelocity);


        if(doLeftOperation)
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

        if(doRightOperation)
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

        ControlQueue* tmp2;
        tmp2 = raQueue;
        raQueue = leQueue;
        leQueue = tmp2;

        cout << "(main) connection to arms established" << endl;

        if(doRightOperation) {
            raQueue->stopCurrentMode();
        //    raQueue->stopCurrentMode();
        //    raQueue->setStiffness(2000, 150, 0.7, 7.0, 70, 2.0);
        //    raQueue->setStiffness(4000, 250, 0.7, 7.0, 70, 2.0);
        //    raQueue->switchMode(30);
            raQueue->switchMode(10);
        }

        cout << "(main) right arm mode switched" << endl;

        if(doLeftOperation) {
            leQueue->stopCurrentMode();
            leQueue->switchMode(10);
        }

        cout << "(main) left arm mode switched" << endl;

        // move to initial position
        if(doRightOperation)
            raQueue->moveJoints(headInitJoints);

//        if(doLeftOperation)
//            leQueue->moveJoints(pendInitJoints);

        if(doRightOperation)
            raQueue->moveJoints(headGraspJoints);

        if(doLeftOperation)
            leQueue->moveJoints(pendGraspJoints);

        // go to grasp position
        if(doRightOperation) {

            if(useHands)
                raHand->closeHand(1.0, handVelocity);

            raQueue->moveJoints(headIntermedJoints);
        }


        if(doLeftOperation) {
//            leHand->closeHand(1.0, handVelocity);
            leQueue->moveJoints(pendIntermedJoints);
        }

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
        if(doRightOperation && useHands)
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
