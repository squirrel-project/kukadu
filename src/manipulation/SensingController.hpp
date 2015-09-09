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

class SensingController {

private:

    std::string tmpPath;
    std::string classifierPath;
    std::string classifierFile;
    std::string classifierFunction;

    std::vector<std::shared_ptr<GenericHand>> hands;
    std::vector<std::shared_ptr<KukieControlQueue>> queues;

public:

    SensingController(std::vector<std::shared_ptr<KukieControlQueue>> queues, std::vector<std::shared_ptr<GenericHand>> hands,
                      std::string tmpPath, std::string classifierPath, std::string classifierFile, std::string classifierFunction);

    virtual void prepare() = 0;
    virtual void cleanUp() = 0;
    virtual void performCore() = 0;

    int performClassification(int hapticMode, std::string databasePath);
    int callClassifier(std::string trainedPath, std::string passedFilePath);

    std::shared_ptr<ControllerResult> performAction();

    static const int HAPTIC_MODE_TERMINAL = 0;
    static const int HAPTIC_MODE_CLASSIFIER = 1;

};



#endif // SENSINGCONTROLLER_H
