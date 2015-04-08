#ifndef SENSORSTORAGE
#define SENSORSTORAGE

#include <iostream>
#include <cstdlib>
#include <vector>
#include <thread>
#include <armadillo>
#include <memory>
#include <ros/ros.h>

// Custom librairies
#include "ControlQueue.h"
#include "../utils/types.h"
#include "../utils/utils.h"
#include "../types/SensorData.h"
#include "mounted/GenericHand.h"

#define STORE_TIME 64
#define STORE_RBT_JNT_POS 1
#define STORE_RBT_CART_POS 2
#define STORE_RBT_JNT_FTRQ 4
#define STORE_RBT_CART_FTRQ 8
#define STORE_HND_JNT_POS 16
#define STORE_HND_TCTLE 32

class SensorStorage {

private:

    bool stopped;
    bool storageStopped;

    bool storeTime;
    bool storeJntPos;
    bool storeCartPos;
    bool storeJntFrc;
    bool storeCartFrcTrq;
    bool storeHndJntPos;
    bool storeHndTctle;

    double pollingFrequency;

    std::vector<std::shared_ptr<ControlQueue>> queues;
    std::vector<std::shared_ptr<GenericHand>> hands;

    std::vector<std::shared_ptr<std::ofstream>> queueStreams;
    std::vector<std::shared_ptr<std::ofstream>> handStreams;

    void store();
    void writeVectorInLine(std::shared_ptr<std::ofstream> stream, arma::vec writeVec);
    void writeMatrixInLine(std::shared_ptr<std::ofstream> stream, arma::mat writeMat);
    void writeLabels(std::shared_ptr<std::ofstream> stream, std::vector<std::string> labels);
    void writeMatrixMetaInfo(std::shared_ptr<std::ofstream> stream, int matrixNum, int xDim, int yDim);

public:

    SensorStorage(std::vector<std::shared_ptr<ControlQueue>> queues, std::vector<std::shared_ptr<GenericHand>> hands, double pollingFrequency);
    std::shared_ptr<std::thread> startDataStorage(std::string folderName);

    void stopDataStorage();
    void storeData(bool storeHeader, std::string file, std::shared_ptr<SensorData> data);

    static std::shared_ptr<SensorData> readStorage(std::shared_ptr<ControlQueue> queue, std::string file);
    void storeData(bool storeHeader, std::vector<std::string> files, std::vector<std::shared_ptr<SensorData>> data);
    void storeData(bool storeHeader, std::vector<std::shared_ptr<std::ofstream>> queueStreams, std::vector<std::shared_ptr<SensorData>> data);

    void setExportMode(int mode);

};

#endif
