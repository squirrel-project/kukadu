#ifndef KUKADU_COMPLEXCONTROLLER_H
#define KUKADU_COMPLEXCONTROLLER_H

#include <cstdio>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <wordexp.h>
#include <kukadu/manipulation/controller.hpp>
#include <kukadu/types/controllerresult.hpp>
#include <kukadu/manipulation/sensingcontroller.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/robot/sensorstorage.hpp>
#include <kukadu/robot/arm/kukiecontrolqueue.hpp>
#include <kukadu/learning/projective_simulation/core/reward.hpp>
#include <kukadu/learning/projective_simulation/core/projectivesimulator.hpp>
#include <kukadu/learning/projective_simulation/application/manualreward.hpp>

namespace kukadu {

    class ComplexController : public Controller, public Reward, public KUKADU_ENABLE_SHARED_FROM_THIS<ComplexController> {

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

        std::string storePath;
        std::string rewardHistoryPath;

        KUKADU_SHARED_PTR<PerceptClip> root;
        KUKADU_SHARED_PTR<kukadu_mersenne_twister> gen;
        KUKADU_SHARED_PTR<ProjectiveSimulator> projSim;
        KUKADU_SHARED_PTR<std::ofstream> rewardHistoryStream;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<Clip> > > prepActions;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > prepActionsCasted;

        KUKADU_DISCRETE_DISTRIBUTION<int> simSuccDist;

        std::vector<double> sensingWeights;
        std::vector<KUKADU_SHARED_PTR<Controller> > preparationControllers;
        std::vector<KUKADU_SHARED_PTR<SensingController> > sensingControllers;

        std::map<std::string, KUKADU_SHARED_PTR<kukadu::SensingController> > availableSensingControllers;
        std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > availablePreparatoryControllers;

        // novelty in icdl paper
        std::map<std::string, KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> > environmentModels;

        double computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction);

        KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> createEnvironmentModelForSensingAction(KUKADU_SHARED_PTR<kukadu::SensingController> sensingAction, KUKADU_SHARED_PTR<ProjectiveSimulator> projSim);

    protected:

        void setSimulationModeInChain(bool simulationMode);
        virtual double getSimulatedReward(KUKADU_SHARED_PTR<SensingController> usedSensingController, KUKADU_SHARED_PTR<kukadu::PerceptClip> providedPercept, KUKADU_SHARED_PTR<kukadu::Controller> takenAction, int sensingClassIdx, int prepContIdx) = 0;

        virtual double getSimulatedRewardInternal(KUKADU_SHARED_PTR<SensingController> usedSensingController, KUKADU_SHARED_PTR<kukadu::PerceptClip> providedPercept, KUKADU_SHARED_PTR<kukadu::Controller> takenAction, int sensingClassIdx, int prepContIdx);

    public:

        ComplexController(std::string caption, std::string storePath,
                          bool storeReward, double senseStretch, double boredom, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator,
                          int stdReward, double punishReward, double gamma, int stdPrepWeight, bool collectPrevRewards, int simulationFailingProbability);
        ~ComplexController();

        void store();
        void load(std::string path, std::map<std::string, KUKADU_SHARED_PTR<kukadu::SensingController> > availableSensingControllers, std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > availablePreparatoryControllers);

        virtual void initialize();
        void storeNextIteration();
        void createSensingDatabase();
        void setBoredom(double boredom);
        void store(std::string destination);
        void setTrainingMode(bool doTraining);
        void createSensingDatabase(std::vector<KUKADU_SHARED_PTR<SensingController> > sensingControllers);

        void setSensingControllers(std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > sensingControllers);
        void setPreparatoryControllers(std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > preparatoryControllers);

        virtual void executeComplexAction() = 0;

        int getDimensionality();

        // overwrite this virtual function if the next idx should be created randomly
        virtual int getNextSimulatedGroundTruth(KUKADU_SHARED_PTR<SensingController> sensCont);

        double getStdReward();
        double getPunishReward();

        KUKADU_SHARED_PTR<ControllerResult> performAction();
        KUKADU_SHARED_PTR<ProjectiveSimulator> getProjectiveSimulator();
        KUKADU_SHARED_PTR<PerceptClip> generateNextPerceptClip(int immunity);
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > generateActionClips();
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > generatePerceptClips();

#ifdef USEBOOST
        static const std::string FILE_SENSING_PREFIX;
        static const std::string FILE_PREP_PREFIX;
        static const std::string FILE_END_PREFIX;
#else
        static constexpr auto FILE_SENSING_PREFIX = "***sensing controllers:";
        static constexpr auto FILE_PREP_PREFIX = "***preparatory controllers:";
        static constexpr auto FILE_END_PREFIX = "***end";
#endif

    };

}

#endif
