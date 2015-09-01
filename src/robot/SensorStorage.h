#ifndef SENSORSTORAGE
#define SENSORSTORAGE

#include <iostream>
#include <cstdlib>
#include <vector>
#include <thread>
#include <armadillo>
#include <memory>
#include <ros/ros.h>
#include <string>

// Custom librairies
#include "ControlQueue.h"
#include "../utils/types.h"
#include "../utils/utils.h"
#include "../types/SensorData.h"
#include "mounted/GenericHand.h"
#include "SimInterface.h"
#include "VisionInterface.h"

#define STORE_TIME 512
#define STORE_RBT_JNT_POS 1
#define STORE_RBT_CART_POS 2
#define STORE_RBT_JNT_FTRQ 4
#define STORE_RBT_CART_FTRQ 8
#define STORE_HND_JNT_POS 16
#define STORE_HND_TCTLE 32
#define STORE_CART_ABS_FRC 64
#define STORE_SIM_OBJECT 128
#define STORE_VIS_OBJECT 256

class SensorStorage {

private:

    bool vision;
    bool stopped;
    bool storeTime;
    bool simulation;
    bool storeJntPos;
    bool storeJntFrc;
    bool storeCartPos;
    bool storeHndTctle;
    bool storageStopped;
    bool storeHndJntPos;
    bool storeSimObject;
    bool storeVisObject;
    bool storeCartFrcTrq;
    bool storeCartAbsFrc;

    double pollingFrequency;

    std::string objectID;

    std::shared_ptr<std::thread> thr;

    std::vector<std::shared_ptr<GenericHand>> hands;
    std::vector<std::shared_ptr<ControlQueue>> queues;

    std::shared_ptr<SimInterface> sim;
    std::shared_ptr<VisionInterface> vis;

    std::shared_ptr<std::ofstream> simStream;
    std::shared_ptr<std::ofstream> visStream;
    std::vector<std::shared_ptr<std::ofstream>> queueStreams;
    std::vector<std::shared_ptr<std::ofstream>> handStreams;

    void store();
    void writeVectorInLine(std::shared_ptr<std::ofstream> stream, arma::vec writeVec);
    void writeMatrixInLine(std::shared_ptr<std::ofstream> stream, arma::mat writeMat);
    void writeLabels(std::shared_ptr<std::ofstream> stream, std::vector<std::string> labels);
    void writeMatrixMetaInfo(std::shared_ptr<std::ofstream> stream, int matrixNum, int xDim, int yDim);
    void initSensorStorage(bool simulation, bool useVision, std::shared_ptr<SimInterface> simInterface, std::string objectId, std::vector<std::shared_ptr<ControlQueue>> queues, std::vector<std::shared_ptr<GenericHand>> hands, std::shared_ptr<VisionInterface> vis, double pollingFrequency, bool storeSimObject, bool storeVisObject);

public:

    SensorStorage(std::vector<std::shared_ptr<ControlQueue>> queues, std::vector<std::shared_ptr<GenericHand>> hands, double pollingFrequency);
    SensorStorage(std::vector<std::shared_ptr<ControlQueue>> queues, std::vector<std::shared_ptr<GenericHand>> hands, std::shared_ptr<SimInterface> sim, std::string objectID, double pollingFrequency);
    SensorStorage(std::vector<std::shared_ptr<ControlQueue>> queues, std::vector<std::shared_ptr<GenericHand>> hands, std::shared_ptr<VisionInterface> vis, double pollingFrequency);

    void stopDataStorage();
    void setExportMode(int mode);
    void storeData(bool storeHeader, std::string file, std::shared_ptr<SensorData> data);
    void storeData(bool storeHeader, std::vector<std::string> files, std::vector<std::shared_ptr<SensorData>> data);
    void storeData(bool storeHeader, std::vector<std::shared_ptr<std::ofstream>> queueStreams, std::vector<std::shared_ptr<SensorData>> data);

    std::shared_ptr<std::thread> startDataStorage(std::string folderName);
    static std::shared_ptr<SensorData> readStorage(std::shared_ptr<ControlQueue> queue, std::string file);

};

#endif
