#include "RosSchunk.h"

using namespace std;

RosSchunk::RosSchunk(ros::NodeHandle node, string trajTopic, std::string stateTopic, string hand) {

    this->node = node;
    // trajPub = node.advertise<motion_control_msgs::JointPositions>(trajTopic, 1);
    trajPub = node.advertise<std_msgs::Float64MultiArray>(trajTopic, 1);

    stateSub = node.subscribe(stateTopic, 1, &RosSchunk::stateCallback, this);
    previousCurrentPosQueueSize = 10;
    isFirstCallback = true;

    currentGraspId = SDH::cSDHBase::eGID_CENTRICAL;
    closeHand(0.0, 1.0);

}

void RosSchunk::stateCallback(const sensor_msgs::JointState state) {

    currentPosMutex.lock();

        if(!isFirstCallback) {
            previousCurrentPosQueue.erase(previousCurrentPosQueue.begin(), previousCurrentPosQueue.begin() + 1);
            previousCurrentPosQueue.push_back(currentPos);
            currentPos = state.position;

        } else {

            previousCurrentPosQueue.clear();
            for(int i = 0; i < previousCurrentPosQueueSize; ++i)
                previousCurrentPosQueue.push_back(state.position);

            currentPos = state.position;
            isFirstCallback = false;

        }

        vector<double> doubleCommandedPos;
        for(int i = 0; i < currentCommandedPos.size(); ++i)
            doubleCommandedPos.push_back(currentCommandedPos.at(i));


        targetReached = !vectorsDeviate(state.position, doubleCommandedPos, 0.01);

        if(!targetReached) {

            bool stillMoving = false;
            for(int i = 0; i < previousCurrentPosQueue.size(); ++i) {
                vector<double> previousCurrentPos = previousCurrentPosQueue.at(i);
                bool deviates = vectorsDeviate(previousCurrentPos, currentPos, 0.01);
                if(deviates) {
                    stillMoving = true;
                    break;
                }
            }

            if(!movementStarted && stillMoving) {
                movementStarted = true;
            } else if(movementStarted && !stillMoving) {
                targetReached = true;
            }

        }

    currentPosMutex.unlock();

}

bool RosSchunk::vectorsDeviate(const std::vector<double> v1, const std::vector<double> v2, double tolerance) {

    bool tmpReached = true;

    for(int i = 0; i < v1.size(); ++i) {
        if(abs(v1.at(i) - v2.at(i)) > tolerance)
            tmpReached = false;
    }

    return !tmpReached;

}

std::vector<float> RosSchunk::generateCylindricalPose(double percentage) {

    vector<float> hand_pose;

    hand_pose.push_back(0);
    hand_pose.push_back((-30 + percentage * 30) * M_PI / 180.0);
    hand_pose.push_back((30 + percentage * 35) * M_PI / 180.0);
    hand_pose.push_back((-30 + percentage * 30) * M_PI / 180.0);
    hand_pose.push_back((30 + percentage * 35) * M_PI / 180.0);
    hand_pose.push_back((-30 + percentage * 30) * M_PI / 180.0);
    hand_pose.push_back((30 + percentage * 35) * M_PI / 180.0);

    return hand_pose;

}

std::vector<float> RosSchunk::generateParallelPose(double percentage) {

    vector<float> hand_pose;

    // finger orientation
    hand_pose.push_back(0);

    // finger 1
    hand_pose.push_back((-75. + percentage * 82.) * M_PI / 180.0);


    hand_pose.push_back((75. - percentage * 82.) * M_PI / 180.0);

    // finger 2, joint 1
    hand_pose.push_back((-75. + percentage * 82.) * M_PI / 180.0);

    // finger 2, joint 2
    hand_pose.push_back((90. - percentage * 82.) * M_PI / 180.0);

    hand_pose.push_back((-75. + percentage * 82.) * M_PI / 180.0);
    hand_pose.push_back((90. - percentage * 82.) * M_PI / 180.0);

    return hand_pose;

}

std::vector<float> RosSchunk::generateCentricalPose(double percentage) {

    vector<float> hand_pose;

    hand_pose.push_back((60) * M_PI/180.0);
    hand_pose.push_back((-75 + percentage * 82) * M_PI / 180.0);
    hand_pose.push_back((75 - percentage * 82) * M_PI / 180.0);
    hand_pose.push_back((-75 + percentage * 82) * M_PI / 180.0);
    hand_pose.push_back((75 - percentage * 82) * M_PI / 180.0);
    hand_pose.push_back((-75 + percentage * 82) * M_PI / 180.0);
    hand_pose.push_back((75 - percentage * 82) * M_PI / 180.0);

    return hand_pose;

}

std::vector<float> RosSchunk::generateSphericalPose(double percentage) {

    vector<float> hand_pose;

    hand_pose.push_back((60) * M_PI / 180.0);
    hand_pose.push_back((-40 + percentage * 25) * M_PI / 180.0);
    hand_pose.push_back((40 + percentage * 15) * M_PI / 180.0);
    hand_pose.push_back((-40 + percentage * 25) * M_PI / 180.0);
    hand_pose.push_back((40 + percentage * 15) * M_PI / 180.0);
    hand_pose.push_back((-40 + percentage * 25) * M_PI / 180.0);
    hand_pose.push_back((40 + percentage * 15) * M_PI / 180.0);

    return hand_pose;

}

void RosSchunk::connectHand() {
    // nothing to do when using ros
}

void RosSchunk::closeHand(double percentage, double velocity) {

    if(percentage >= 0.0 && percentage <= 1.1) {

        vector<float> hand_pose;

        switch(currentGraspId) {
        case SDH::cSDHBase::eGID_CENTRICAL:
            hand_pose = generateCentricalPose(percentage);
            break;
        case SDH::cSDHBase::eGID_CYLINDRICAL:
            hand_pose = generateParallelPose(percentage);
            break;
        case SDH::cSDHBase::eGID_PARALLEL:
            hand_pose = generateParallelPose(percentage);
            break;
        case SDH::cSDHBase::eGID_SPHERICAL:
            hand_pose = generateSphericalPose(percentage);
            break;
        default:

            string msg = "(RosSchunk) grasp type not supported";
            cerr << msg << endl;
            throw msg;

        }

        publishSdhJoints(hand_pose);

    } else {
        string msg = "(RosSchunk) grasp percentage out of range";
        cerr << msg << endl;
        throw msg;
    }

}

void RosSchunk::disconnectHand() {

    // nothing to do when using ros

}

void RosSchunk::setGrasp(SDH::cSDHBase::eGraspId grasp) {

    closeHand(0.0, 1.0);
    currentGraspId = grasp;

}

void RosSchunk::safelyDestroy() {

}

void RosSchunk::publishSdhJoints(std::vector<float> positions) {

    throw "reimplement rosschunk";
    /*
        motion_control_msgs::JointPositions newJoints;
        for(int i = 0; i < positions.size(); ++i)
            newJoints.positions.push_back(positions.at(i));

        currentCommandedPos = positions;

        // get current position
        ros::spinOnce();

        targetReached = false;
        movementStarted = false;

        initCurrentPos = currentPos;

        int nr_trial = 3;
        for(int i = 0; i < 3; i++ ){

                trajPub.publish(newJoints);
                usleep(500*1000);

        }

        while(!targetReached)
            ros::spinOnce();
*/
}
