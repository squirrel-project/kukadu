#include "ComplexController.hpp"

#include "haptic/ControllerActionClip.h"
#include "haptic/IntermediateEventClip.h"

#include "../learning/projective_simulation/core/clip.h"
#include "../learning/projective_simulation/core/perceptclip.h"

using namespace std;

ComplexController::ComplexController(std::string caption, std::vector<std::shared_ptr<SensingController>> sensingControllers,
                                     std::vector<std::shared_ptr<Controller>> preparationControllers,
                                     std::string corrPSPath, std::shared_ptr<std::mt19937> generator, int stdReward, double gamma, int stdPrepWeight, bool collectPrevRewards)
    : Controller(caption), Reward(generator, collectPrevRewards) {

    projSim = nullptr;

    this->gamma = gamma;
    this->gen = generator;
    this->colPrevRewards = collectPrevRewards;

    this->stdReward = stdReward;
    this->corrPSPath = corrPSPath;
    this->stdPrepWeight = stdPrepWeight;
    this->sensingControllers = sensingControllers;
    this->preparationControllers = preparationControllers;

}

void ComplexController::initialize() {

    bool existsPs = fileExists(corrPSPath);
    createSensingDatabase();

    int currentId = 0;
    shared_ptr<vector<int>> clipDimVal = shared_ptr<vector<int>>(new vector<int>());
    clipDimVal->push_back(currentId);
    root = shared_ptr<PerceptClip>(new PerceptClip(0, "root", generator, clipDimVal, INT_MAX));

    vector<double> prepWeights;
    prepActions = shared_ptr<vector<shared_ptr<Clip>>>(new vector<shared_ptr<Clip>>());
    prepActionsCasted = shared_ptr<vector<shared_ptr<ActionClip>>>(new vector<shared_ptr<ActionClip>>());
    for(int i = 0; i < preparationControllers.size(); ++i) {

        shared_ptr<ActionClip> prepActionClip = shared_ptr<ActionClip>(new ControllerActionClip(i, preparationControllers.at(i), generator));
        prepActions->push_back(prepActionClip);
        prepActionsCasted->push_back(prepActionClip);
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
        root->addSubClip(nextSensClip, sensingWeights.at(i) * 100);

    }

    if(existsPs) {
        // skill was already initialized and can be loaded again
        projSim = shared_ptr<ProjectiveSimulator>(new ProjectiveSimulator(shared_from_this(), generator, corrPSPath));
    } else {

        shared_ptr<vector<shared_ptr<PerceptClip>>> rootVec = shared_ptr<vector<shared_ptr<PerceptClip>>>(new vector<shared_ptr<PerceptClip>>());
        rootVec->push_back(root);

        // skill is used the first time; do initialization
        projSim = shared_ptr<ProjectiveSimulator>(new ProjectiveSimulator(shared_from_this(), generator, rootVec, gamma, PS_USE_ORIGINAL, false));

    }

}

std::shared_ptr<ProjectiveSimulator> ComplexController::getProjectiveSimulator() {
    return projSim;
}

void ComplexController::store() {
    projSim->storePS(corrPSPath);
}

int ComplexController::getDimensionality() {
    cout << "(ComplexController) get dimensionality got called" << endl;
    return 1;
}

// same percept must always have same id
std::shared_ptr<PerceptClip> ComplexController::generateNextPerceptClip(int immunity) {
    return root;
}

std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> ComplexController::generateActionClips() {
    return prepActionsCasted;
}

std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>> ComplexController::generatePerceptClips() {
    return std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>>(new std::vector<std::shared_ptr<PerceptClip>>({root}));
}

double ComplexController::computeRewardInternal(std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction) {

    int worked = 0;
    int executeIt = 0;
    double retReward = 0.0;

    shared_ptr<vector<int>> intermed = projSim->getIntermediateHopIdx();
    shared_ptr<Clip> sensClip = providedPercept->getSubClipByIdx(0)->getSubClipByIdx(intermed->at(0));

    cout << "selected sensing action \"" << *sensClip << "\" resulted in preparation action \"" << *takenAction << "\"" << endl;

    shared_ptr<ControllerActionClip> castedAction = dynamic_pointer_cast<ControllerActionClip>(takenAction);
    castedAction->performAction();

    cout << "(ComplexController) do you want to execute complex action now? (0 = no / 1 = yes)" << endl;
    cin >> executeIt;

    if(executeIt) {
        executeComplexAction();
    }

    cout << "did the complex action succeed? (0 = no / 1 = yes)" << endl;
    cin >> worked;

    if(worked) {
        cout << "preparation action worked; rewarded with " << stdReward << endl;
        retReward = stdReward;
    } else {
        cout << "preparation action didn't work; no reward given" << endl;
        retReward = 0.0;
    }

    return retReward;

}

std::shared_ptr<ControllerResult> ComplexController::performAction() {
    projSim->performRandomWalk();
    projSim->performRewarding();
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
