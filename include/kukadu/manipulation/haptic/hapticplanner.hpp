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
#include <kukadu/utils/utils.hpp>
#include <kukadu/types/controllerresult.hpp>
#include <kukadu/manipulation/controller.hpp>
#include <kukadu/manipulation/sensingcontroller.hpp>
#include <kukadu/learning/projective_simulation/core/projectivesimulator.hpp>

namespace kukadu {

    class HapticPlanner : public kukadu::Reward, public KUKADU_ENABLE_SHARED_FROM_THIS<HapticPlanner> {

    private:

        std::string skillDatabase;

        std::map<std::string, KUKADU_SHARED_PTR<kukadu::SensingController> > registeredSensingControllers;
        std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > registeredPrepControllers;
        std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > registeredComplexControllers;

        KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator;

        void printNamedVector(std::vector<std::string> names);

        std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > copySensingControllers(std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > controllers,
                                                                                          std::string newBasePath);

    protected:

        virtual double computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction);

    public:

        HapticPlanner(std::string skillDatabase,
                      std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > sensingControllers,
                      std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > preparatoryControllers,
                      std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > complexControllers,
                      KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator);

        virtual int getDimensionality();

        virtual KUKADU_SHARED_PTR<PerceptClip> generateNextPerceptClip(int immunity);
        virtual KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > generateActionClips();
        virtual KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > generatePerceptClips();

        void pickAndPerformComplexSkill();
        KUKADU_SHARED_PTR<kukadu::HapticControllerResult> performComplexSkill(std::string skillId);

        std::string pickComplexSkill();

    };

}

#endif // HAPTICPLANNER_H
