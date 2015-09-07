#ifndef HAPTICPLANNER_H
#define HAPTICPLANNER_H

#include "../Controller.hpp"
#include "../../../kukadu/src/utils/utils.h"

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

public:

    void addSensingController(std::string name);
    void addPreparationController(std::string name, std::shared_ptr<Controller> prepController);
    void addComplexController(std::string name, std::shared_ptr<Controller> complexController);

};



#endif // HAPTICPLANNER_H
