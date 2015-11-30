#ifndef SENSORSTORAGE
#define SENSORSTORAGE

#include <string>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <armadillo>
#include <ros/ros.h>

// Custom librairies
#include "../utils/types.h"
#include "../utils/utils.h"
#include "VisionInterface.h"
#include "../types/SensorData.h"
#include "mounted/GenericHand.h"
#include "../types/KukaduTypes.h"
#include "../robot/ControlQueue.h"

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

    bool stopped;
    bool storeTime;
    bool storeJntPos;
    bool storeJntFrc;
    bool storeCartPos;
    bool storeHndTctle;
    bool storageStopped;
    bool storeHndJntPos;
    bool storeCartFrcTrq;
    bool storeCartAbsFrc;

    double pollingFrequency;

    KUKADU_SHARED_PTR<kukadu_thread> thr;

    std::vector<KUKADU_SHARED_PTR<GenericHand> > hands;
    std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues;

    std::vector<KUKADU_SHARED_PTR<std::ofstream> > handStreams;
    std::vector<KUKADU_SHARED_PTR<std::ofstream> > queueStreams;

    void store();
    void writeVectorInLine(KUKADU_SHARED_PTR<std::ofstream> stream, arma::vec writeVec);
    void writeMatrixInLine(KUKADU_SHARED_PTR<std::ofstream> stream, arma::mat writeMat);
    void writeLabels(KUKADU_SHARED_PTR<std::ofstream> stream, std::vector<std::string> labels);
    void writeMatrixMetaInfo(KUKADU_SHARED_PTR<std::ofstream> stream, int matrixNum, int xDim, int yDim);
    void initSensorStorage(std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, std::vector<KUKADU_SHARED_PTR<GenericHand> > hands, double pollingFrequency);

public:

    SensorStorage(std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, std::vector<KUKADU_SHARED_PTR<GenericHand> > hands, double pollingFrequency);

    void stopDataStorage();
    void setExportMode(int mode);
    void storeData(bool storeHeader, std::string file, KUKADU_SHARED_PTR<SensorData> data);
    void storeData(bool storeHeader, std::vector<std::string> files, std::vector<KUKADU_SHARED_PTR<SensorData> > data);
    void storeData(bool storeHeader, std::vector<KUKADU_SHARED_PTR<std::ofstream> > queueStreams, std::vector<KUKADU_SHARED_PTR<SensorData> > data);

    KUKADU_SHARED_PTR<kukadu_thread> startDataStorage(std::string folderName);
    static KUKADU_SHARED_PTR<SensorData> readStorage(KUKADU_SHARED_PTR<ControlQueue> queue, std::string file);

};

#endif
