#ifndef HAPTICPLANNER_H
#define HAPTICPLANNER_H

#include "../Controller.hpp"
#include "../SensingController.hpp"

#include "../../utils/utils.h"

#include <cstdio>
#include <iostream>
#include <fstream>
#include <thread>
#include <string>
#include <vector>
#include <wordexp.h>
#include <memory>
#include "ros/ros.h"
#include "std_msgs/Int32.h"

class HapticPlanner {

private:

    std::vector<std::string> sensingNames;
    std::vector<std::string> preparationNames;
    std::vector<std::string> complexControllerNames;

    std::vector<std::shared_ptr<SensingController>> sensingControllers;
    std::vector<std::shared_ptr<Controller>> preparationControllers;
    std::vector<std::shared_ptr<Controller>> complexControllers;

public:

    HapticPlanner();

    void addSensingController(std::string name, std::shared_ptr<SensingController> sensingController);
    void addPreparationController(std::string name, std::shared_ptr<Controller> prepController);
    void addComplexController(std::string name, std::shared_ptr<Controller> complexController);

};



#endif // HAPTICPLANNER_H
