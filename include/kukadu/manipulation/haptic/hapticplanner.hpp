#ifndef KUKADU_HAPTICPLANNER_H
#define KUKADU_HAPTICPLANNER_H

#include <cstdio>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <wordexp.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <kukadu/manipulation/controller.hpp>
#include <kukadu/utils/utils.hpp>
#include <kukadu/manipulation/sensingcontroller.hpp>

namespace kukadu {

    class HapticPlanner {

    private:

        std::string skillDatabase;

        std::map<std::string, KUKADU_SHARED_PTR<kukadu::SensingController> > registeredSensingControllers;
        std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > registeredPrepControllers;
        std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > registeredComplexControllers;

        void printNamedVector(std::vector<std::string> names);

        std::string pickComplexController();

    public:

        HapticPlanner(std::string skillDatabase,
                      std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > sensingControllers,
                      std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > preparatoryControllers,
                      std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > complexControllers);

        void performSkill(std::string skillIdx);

    };

}

#endif // HAPTICPLANNER_H
