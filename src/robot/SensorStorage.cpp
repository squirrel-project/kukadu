#include "SensorStorage.h"

#include "../utils/easyloggingpp/src/easylogging++.h"
_INITIALIZE_EASYLOGGINGPP

using namespace std;
using namespace arma;

SensorStorage::SensorStorage(std::vector<std::shared_ptr<ControlQueue>> queues, std::vector<std::shared_ptr<GenericHand>> hands, double pollingFrequency) {

    this->queues = queues;
    this->hands = hands;

    this->pollingFrequency = pollingFrequency;

    stopped = false;
    storageStopped = true;

}

std::shared_ptr<std::thread> SensorStorage::startDataStorage(std::string folderName) {

    if(createDirectory(folderName)) {

        queueStreams.clear();
        for(int i = 0; i < queues.size(); ++i) {
            stringstream s;
            s << i;
            std::shared_ptr<std::ofstream> queueFile = std::shared_ptr<std::ofstream>(new ofstream());
            queueFile->open(folderName + string("/") + queues.at(i)->getRobotFileName() + string("_") + s.str());
            queueStreams.push_back(queueFile);
        }

        handStreams.clear();
        for(int i = 0; i < hands.size(); ++i) {
            stringstream s;
            s << i;
            std::shared_ptr<std::ofstream> queueFile = std::shared_ptr<std::ofstream>(new ofstream());
            queueFile->open(folderName + string("/") + hands.at(i)->getHandName() + string("_") + s.str());
            handStreams.push_back(queueFile);
        }

        shared_ptr<thread> thr = shared_ptr<thread>(nullptr);
        thr = std::shared_ptr<std::thread>(new std::thread(&SensorStorage::storeData, this));
        storageStopped = false;
        return thr;

    } else {

        LOG(WARNING) << "(SensorStorage) directory already exists (no data recorded)";
        return shared_ptr<thread>(nullptr);

    }

}

void SensorStorage::stopDataStorage() {

    stopped = true;

    ros::Rate r(40);
    while(!storageStopped)
        r.sleep();

    for(int i = 0; i < queueStreams.size(); ++i)
        queueStreams.at(i)->close();
    queueStreams.clear();

    for(int i = 0; i < handStreams.size(); ++i)
        handStreams.at(i)->close();
    handStreams.clear();

}
void SensorStorage::writeVectorInLine(std::shared_ptr<ofstream> stream, arma::vec writeVec) {
    for(int i = 0; i < writeVec.n_elem; ++i)
        *stream << writeVec(i) << "\t";
}

void SensorStorage::writeMatrixInLine(std::shared_ptr<ofstream> stream, arma::mat writeMat) {
    for(int i = 0; i < writeMat.n_rows; ++i)
        for(int j = 0; j < writeMat.n_cols; ++j)
            *stream << writeMat(i, j) << "\t";
}

void SensorStorage::storeData() {

    double waitTime = 1.0 / pollingFrequency;
    ros::Rate rate(pollingFrequency);
    double currentTime = 0.0;

    while(!stopped) {

        for(int i = 0; i < queues.size(); ++i) {

            shared_ptr<ControlQueue> currentQueue = queues.at(i);
            shared_ptr<ofstream> currentOfStream = queueStreams.at(i);
            mes_result joints = currentQueue->getCurrentJoints();
            mes_result cartPos = currentQueue->getCartesianPos();
            mes_result jntFrcTrq = currentQueue->getCurrentJntFrcTrq();
            mes_result cartFrcTrq = currentQueue->getCurrentCartesianFrcTrq();

            *currentOfStream << joints.time << "\t";
            writeVectorInLine(currentOfStream, joints.joints);
            writeVectorInLine(currentOfStream, cartPos.joints);
            writeVectorInLine(currentOfStream, jntFrcTrq.joints);
            writeVectorInLine(currentOfStream, cartFrcTrq.joints);
            *currentOfStream << endl;

            currentTime = joints.time;

        }

        for(int i = 0; i < hands.size(); ++i) {

            shared_ptr<GenericHand> currentHand = hands.at(i);
            shared_ptr<ofstream> currentOfStream = handStreams.at(i);
            std::vector<arma::mat> currentSensing = currentHand->getTactileSensing();

            *currentOfStream << currentTime << "\t";
            for(int j = 0; j < currentSensing.size(); ++j)
                writeMatrixInLine(currentOfStream, currentSensing.at(i));

            *currentOfStream << endl;

            if(queues.size() == 0)
                currentTime += waitTime;

        }

        rate.sleep();

    }

    storageStopped = true;

}
