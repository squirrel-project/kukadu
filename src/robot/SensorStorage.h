#ifndef SENSORSTORAGE
#define SENSORSTORAGE

#include <iostream>
#include <cstdlib>
#include <vector>
#include <thread>
#include <armadillo>
#include <ros/ros.h>

// Custom librairies
#include "ControlQueue.h"
#include "../utils/types.h"
#include "../utils/utils.h"
#include "mounted/GenericHand.h"

#define STORE_RBT_JNT_POS 1
#define STORE_RBT_CART_POS 2
#define STORE_RBT_JNT_FTRQ 4
#define STORE_RBT_CART_FTRQ 8
#define STORE_HND_JNT_POS 16
#define STORE_HND_TCTLE 32

/** \brief The OrocosControlQueue provides control capabilities for the Kuka LWR 4+ robotic arm
 *
 * This class implements the abstract ControlQueue class for the usage with the iisorocos system. It provides basic functionalities such as command mode control
 * in joint space as well as point to point movement in cartesian and joint space. To use it, the additionally provided KRL script has to be selected on the robot
 * controller side. For further information how to use it, please see the sample programs and the kuka documentation
 * \ingroup RobotFramework
 */
class SensorStorage {

private:

    bool stopped;

    double pollingFrequency;

    std::vector<std::shared_ptr<ControlQueue>> queues;
    std::vector<std::shared_ptr<GenericHand>> hands;

    std::shared_ptr<std::ofstream> outputStream;

    void storeData();

public:

    SensorStorage(std::vector<std::shared_ptr<ControlQueue>> queues, std::vector<std::shared_ptr<GenericHand>> hands, double pollingFrequency);
    std::shared_ptr<std::thread> startDataStorage(std::string folderName);

};

#endif
