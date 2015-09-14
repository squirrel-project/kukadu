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
#include "../learning/projective_simulation/core/reward.h"
#include "../learning/projective_simulation/core/projectivesimulator.h"
#include "../learning/projective_simulation/application/manualreward.h"

class ComplexController : public Controller, public Reward, public std::enable_shared_from_this<ComplexController> {

private:

    bool colPrevRewards;

    int stdPrepWeight;

    double gamma;
    double stdReward;

    std::string corrPSPath;

    std::shared_ptr<std::mt19937> gen;

    std::shared_ptr<ProjectiveSimulator> projSim;

    std::vector<double> sensingWeights;

    std::shared_ptr<PerceptClip> root;

    std::vector<std::shared_ptr<Controller>> preparationControllers;
    std::vector<std::shared_ptr<SensingController>> sensingControllers;

    std::shared_ptr<std::vector<std::shared_ptr<Clip>>> prepActions;
    std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> prepActionsCasted;

    double computeRewardInternal(std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction);

public:

    ComplexController(std::string caption, std::vector<std::shared_ptr<SensingController>> sensingControllers, std::vector<std::shared_ptr<Controller>> preparationControllers,
                      std::string corrPSPath, std::shared_ptr<std::mt19937> generator, int stdReward, double gamma, int stdPrepWeight, bool collectPrevRewards);

    void store();
    // needs to be called after constructor
    void initialize();
    void createSensingDatabase();
    void createSensingDatabase(std::vector<std::shared_ptr<SensingController>> sensingControllers);

    std::shared_ptr<ControllerResult> performAction();

    int getDimensionality();

    // same percept must always have same id
    std::shared_ptr<PerceptClip> generateNextPerceptClip(int immunity);

    std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> generateActionClips();
    std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>> generatePerceptClips();

    std::shared_ptr<ProjectiveSimulator> getProjectiveSimulator();

    virtual void executeComplexAction() = 0;

};



#endif // COMPLEXCONTROLLER_H
