#include "ComplexController.hpp"

#include "haptic/ControllerActionClip.h"
#include "haptic/IntermediateEventClip.h"

#include "../learning/projective_simulation/core/clip.h"
#include "../learning/projective_simulation/core/perceptclip.h"

using namespace std;

ComplexController::ComplexController(std::string caption, std::vector<std::shared_ptr<SensingController>> sensingControllers,
                                     std::vector<std::shared_ptr<Controller>> preparationControllers,
                                     std::string corrPSPath, std::shared_ptr<std::mt19937> generator, int stdReward, double gamma, int stdPrepWeight) : Controller(caption) {

    bool existsPs = fileExists(corrPSPath);
    shared_ptr<ProjectiveSimulator> projSim = nullptr;
    shared_ptr<ManualReward> manualRew = shared_ptr<ManualReward>(new ManualReward(generator, preparationControllers.size(), sensingControllers.size(), false, stdReward));

    this->stdPrepWeight = stdPrepWeight;
    this->sensingControllers = sensingControllers;
    this->preparationControllers = preparationControllers;

    createSensingDatabase();

    int currentId = 0;
    shared_ptr<vector<int>> clipDimVal = shared_ptr<vector<int>>(new vector<int>());
    clipDimVal->push_back(currentId);
    shared_ptr<PerceptClip> root = shared_ptr<PerceptClip>(new PerceptClip(0, "root", generator, clipDimVal, INT_MAX));

    vector<double> prepWeights;
    prepActions = shared_ptr<vector<shared_ptr<Clip>>>(new vector<shared_ptr<Clip>>());
    for(int i = 0; i < preparationControllers.size(); ++i) {
        shared_ptr<Clip> prepActionClips = shared_ptr<Clip>(new ControllerActionClip(i, preparationControllers.at(i), generator));
        prepActions->push_back(prepActionClips);
        prepWeights.push_back(stdPrepWeight);
    }
    ++currentId;

    for(int i = 0; i < sensingControllers.size(); ++i) {

        shared_ptr<SensingController> sensCont = sensingControllers.at(i);

        clipDimVal = shared_ptr<vector<int>>(new vector<int>());
        clipDimVal->push_back(currentId);

        shared_ptr<Clip> nextSensClip = shared_ptr<Clip>(new IntermediateEventClip(sensCont, 1, generator, clipDimVal, INT_MAX));
        ++currentId;
        for(int j = 0; j < sensCont->getSensingCatCount(); ++j, ++currentId) {

            clipDimVal = shared_ptr<vector<int>>(new vector<int>());
            clipDimVal->push_back(currentId);
            shared_ptr<Clip> nextSubSensClip = shared_ptr<Clip>(new Clip(2, generator, clipDimVal, INT_MAX));
            nextSubSensClip->setChildren(prepActions, prepWeights);
            nextSensClip->addSubClip(nextSubSensClip, stdPrepWeight);

        }

        // todo: adapt weights (according to sandors classifier evaluation)
        root->addSubClip(nextSensClip, sensingWeights.at(i));

    }

    if(existsPs) {
        // skill was already initialized and can be loaded again
        projSim = shared_ptr<ProjectiveSimulator>(new ProjectiveSimulator(manualRew, generator, corrPSPath));
    } else {
        shared_ptr<vector<shared_ptr<PerceptClip>>> rootVec = shared_ptr<vector<shared_ptr<PerceptClip>>>(new vector<shared_ptr<PerceptClip>>());
        rootVec->push_back(root);
        // skill is used the first time; do initialization
        projSim = shared_ptr<ProjectiveSimulator>(new ProjectiveSimulator(manualRew, generator, rootVec, gamma, PS_USE_ORIGINAL, false));
        projSim->storePS(corrPSPath);
    }

}

std::shared_ptr<ControllerResult> ComplexController::performAction() {

}

void ComplexController::createSensingDatabase() {

    createSensingDatabase(sensingControllers);

}

void ComplexController::createSensingDatabase(std::vector<std::shared_ptr<SensingController>> sensingControllers) {

    sensingWeights.clear();
    for(int i = 0; i < sensingControllers.size(); ++i) {
        double validationScore = sensingControllers.at(i)->createDataBase();
        sensingWeights.push_back(validationScore);
    }

}
