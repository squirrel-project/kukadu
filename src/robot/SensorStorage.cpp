#include "SensorStorage.h"

#include "../utils/easyloggingpp/src/easylogging++.h"
_INITIALIZE_EASYLOGGINGPP

using namespace std;
using namespace arma;

SensorStorage::SensorStorage(std::vector<std::shared_ptr<ControlQueue>> queues, std::vector<std::shared_ptr<GenericHand>> hands, double pollingFrequency) {

    this->queues = queues;
    this->hands = hands;
    this->outputStream = shared_ptr<ofstream>(nullptr);

    this->pollingFrequency = pollingFrequency;

    stopped = false;

}

std::shared_ptr<std::thread> SensorStorage::startDataStorage(std::string folderName) {

    if(createDirectory(folderName)) {

        shared_ptr<thread> thr = shared_ptr<thread>(nullptr);
        this->outputStream = outputStream;
        thr = std::shared_ptr<std::thread>(new std::thread(&SensorStorage::storeData, this));
        return thr;

    } else {

        LOG(WARNING) << "(SensorStorage) directory already exists";

    }

}

void SensorStorage::storeData() {

    ros::Rate rate(pollingFrequency);

    while(!stopped) {

        for(int i = 0; i < queues.size(); ++i) {

            shared_ptr<ControlQueue> currentQueue = queues.at(i);
            mes_result joints = currentQueue->getCurrentJoints();
            mes_result cartPos = currentQueue->getCartesianPos();
            mes_result jntFrcTrq = currentQueue->getCurrentJntFrcTrq();
            mes_result cartFrcTrq = currentQueue->getCurrentCartesianFrcTrq();

        }

        for(int i = 0; i < hands.size(); ++i) {

            shared_ptr<GenericHand> currentHand = hands.at(i);
            std::vector<arma::mat> currentSensing = currentHand->getTactileSensing();

        }

        rate.sleep();

    }

}
