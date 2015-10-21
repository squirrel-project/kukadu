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

    bool storeReward;
    bool colPrevRewards;

    int stdPrepWeight;
    int currentIterationNum;

    double gamma;
    double boredom;
    double stdReward;
    double punishReward;
    double senseStretch;

    std::string corrPSPath;
    std::string rewardHistoryPath;

    std::shared_ptr<std::ofstream> rewardHistoryStream;

    std::shared_ptr<std::mt19937> gen;

    std::shared_ptr<ProjectiveSimulator> projSim;

    std::vector<double> sensingWeights;

    std::shared_ptr<PerceptClip> root;

    std::vector<std::shared_ptr<Controller>> preparationControllers;
    std::vector<std::shared_ptr<SensingController>> sensingControllers;

    std::shared_ptr<std::vector<std::shared_ptr<Clip>>> prepActions;
    std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> prepActionsCasted;

    double computeRewardInternal(std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction);

protected:

    void setSimulationModeInChain(bool simulationMode);
    virtual double getSimulatedReward(std::shared_ptr<SensingController> usedSensingController, std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction, int sensingClassIdx, int prepContIdx) = 0;

public:

    ComplexController(std::string caption, std::vector<std::shared_ptr<SensingController>> sensingControllers, std::vector<std::shared_ptr<Controller>> preparationControllers,
                      std::string corrPSPath, std::string rewardHistoryPath, bool storeReward, double senseStretch, double boredom, std::shared_ptr<std::mt19937> generator, int stdReward, double punishReward, double gamma, int stdPrepWeight, bool collectPrevRewards);
    ~ComplexController();

    void store();
    void storeNextIteration();
    void setBoredom(double boredom);
    void store(std::string destination);
    void setTrainingMode(bool doTraining);

    // needs to be called after constructor
    void initialize();
    void createSensingDatabase();
    void createSensingDatabase(std::vector<std::shared_ptr<SensingController>> sensingControllers);

    std::shared_ptr<ControllerResult> performAction();

    int getDimensionality();

    double getStdReward();
    double getPunishReward();

    // same percept must always have same id
    std::shared_ptr<PerceptClip> generateNextPerceptClip(int immunity);

    std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> generateActionClips();
    std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>> generatePerceptClips();

    std::shared_ptr<ProjectiveSimulator> getProjectiveSimulator();

    virtual void executeComplexAction() = 0;

    // overwrite this virtual function if the next idx should be created randomly
    virtual int getNextSimulatedGroundTruth(std::shared_ptr<SensingController> sensCont);

};



#endif // COMPLEXCONTROLLER_H
