#ifndef COMPLEXCONTROLLER_H
#define COMPLEXCONTROLLER_H

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
#include "SensingController.hpp"
#include "../robot/SensorStorage.h"
#include "../robot/KukieControlQueue.h"
#include "../learning/projective_simulation/core/projectivesimulator.h"
#include "../learning/projective_simulation/application/manualreward.h"

class ComplexController : public Controller {

private:

    std::string corrPSPath;

    std::vector<std::shared_ptr<Controller>> preparationControllers;
    std::vector<std::shared_ptr<SensingController>> sensingControllers;

    void writeLabelFile(std::string baseFolderPath, std::vector<std::pair<int, std::string>> collectedSamples);

public:

    ComplexController(std::string caption, std::vector<std::shared_ptr<SensingController>> sensingControllers, std::vector<std::shared_ptr<Controller>> preparationControllers, std::string corrPSPath, std::shared_ptr<std::mt19937> generator, int stdReward, double gamma);

    std::vector<std::string> createSensingDatabase();
    std::vector<std::string> createSensingDatabase(std::vector<std::string> databasePaths, std::vector<std::shared_ptr<SensingController>> sensingControllers);

    // returns cross validation score
    double createDataBaseForSingleSense(std::string path, std::shared_ptr<SensingController> sensingController);

    std::shared_ptr<ControllerResult> performAction();

};



#endif // COMPLEXCONTROLLER_H
