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

    int stdPrepWeight;

    std::string corrPSPath;

    std::vector<double> sensingWeights;

    std::vector<std::shared_ptr<Controller>> preparationControllers;
    std::vector<std::shared_ptr<SensingController>> sensingControllers;

    std::shared_ptr<std::vector<std::shared_ptr<Clip>>> prepActions;

public:

    ComplexController(std::string caption, std::vector<std::shared_ptr<SensingController>> sensingControllers, std::vector<std::shared_ptr<Controller>> preparationControllers, std::string corrPSPath, std::shared_ptr<std::mt19937> generator, int stdReward, double gamma, int stdPrepWeight);

    void createSensingDatabase();
    void createSensingDatabase(std::vector<std::shared_ptr<SensingController>> sensingControllers);

    std::shared_ptr<ControllerResult> performAction();

};



#endif // COMPLEXCONTROLLER_H
