#ifndef SENSINGCONTROLLER_H
#define SENSINGCONTROLLER_H

#include <cstdio>
#include <iostream>
#include <fstream>
#include <thread>
#include <string>
#include <vector>
#include <wordexp.h>
#include <memory>

#include "Controller.hpp"
#include "ControllerResult.hpp"
#include "../robot/SensorStorage.h"
#include "../robot/KukieControlQueue.h"

class SensingController : public Controller {

private:

    bool classifierParamsSet;

    int hapticMode;
    int currentIterationNum;

    double bestParamC;
    double bestParamD;
    double bestParamParam1;
    double bestParamParam2;

    std::string tmpPath;
    std::string databasePath;
    std::string classifierPath;
    std::string classifierFile;
    std::string classifierFunction;

    std::vector<std::shared_ptr<GenericHand>> hands;
    std::vector<std::shared_ptr<KukieControlQueue>> queues;

    std::vector<double> callClassifier(std::string trainedPath, std::string passedFilePath, bool classify,
                                       double bestParamC, double bestParamD, double bestParamParam1, double bestParamParam2);

    void writeLabelFile(std::string baseFolderPath, std::vector<std::pair<int, std::string>> collectedSamples);

public:

    SensingController(int hapticMode, std::string caption, std::string databasePath, std::vector<std::shared_ptr<KukieControlQueue>> queues, std::vector<std::shared_ptr<GenericHand>> hands,
                      std::string tmpPath, std::string classifierPath, std::string classifierFile, std::string classifierFunction);

    virtual void prepare() = 0;
    virtual void cleanUp() = 0;
    virtual void performCore() = 0;

    void gatherData(std::string completePath);
    void gatherData(std::string dataBasePath, std::string dataName);

    virtual int getSensingCatCount() = 0;

    // classifies the sensor data that comes from the first passed queue; the file has to end with the "_0" postfix as well
    int performClassification();

    void setCLassifierParams(double bestParamC, double bestParamD, double bestParamParam1, double bestParamParam2);

    std::string getDatabasePath();
    // returns robot file name of the first passed control queue (the one, from which the force data is sampled)
    std::string getFirstRobotFileName();

    std::shared_ptr<ControllerResult> performAction();

    // returns cross validation score
    double createDataBase();

    std::vector<double> callClassifier();

    static const int HAPTIC_MODE_TERMINAL = 0;
    static const int HAPTIC_MODE_CLASSIFIER = 1;

};



#endif // SENSINGCONTROLLER_H
