#include "neverendingcolorreward.h"

#include <sstream>

using namespace std;

NeverendingColorReward::NeverendingColorReward(std::shared_ptr<std::mt19937> generator, int numberOfActions, int numberOfCategories, bool collectPrevRewards) : Reward(generator, collectPrevRewards) {

    if(numberOfActions > 9)
        throw "(NeverEndingColorReward) number of actions must be smaller than 10";

    currentT = 0;
    this->numberOfActions = numberOfActions;
    this->numberOfCategories = numberOfCategories + 1;

    intDist = std::uniform_int_distribution<int>(0, numberOfActions - 1);

    actionClips = new vector<std::shared_ptr<ActionClip>>();
    for(int i = 0; i < numberOfActions; ++i) {
        stringstream s;
        s << "action" << i;
        actionClips->push_back(std::shared_ptr<ActionClip>(new ActionClip(i, this->numberOfCategories, s.str(), generator)));
    }

    perceptClips = new vector<std::shared_ptr<PerceptClip>>();
    currentId = 0;

}

NeverendingColorReward::~NeverendingColorReward() {
    delete actionClips;
    delete perceptClips;
}

std::shared_ptr<PerceptClip> NeverendingColorReward::generateNextPerceptClip(int immunity) {

    int currentColor = currentT;

    stringstream s;
    s << "(color" << currentColor;

    std::shared_ptr<vector<int>> idVec = shared_ptr<vector<int>>(new vector<int>());
    idVec->push_back(currentT);
    for(int i = 1; i < numberOfCategories - 1; ++i) {

        int nextVal = intDist(*generator);
        idVec->push_back(nextVal);
        s << ", action" << nextVal;

    }

    int actionNumber = intDist(*generator);
    s << ", " << actionNumber << ")";
    idVec->push_back(actionNumber);
    return std::shared_ptr<PerceptClip>(new PerceptClip(currentId++, s.str(), generator, idVec, immunity));

}

double NeverendingColorReward::computeRewardInternal(std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction) {

    double returnedReward = 0.0;
    ++currentT;

    std::shared_ptr<vector<int>> perceptIds = providedPercept->getClipDimensions();

    if(perceptIds->at(1) == takenAction->getActionId())
        returnedReward = NEVERENDINGCOLORREWARD_SUCCESSFUL_REWARD;

    return returnedReward;

}

std::shared_ptr<std::vector<std::shared_ptr<ActionClip> >> NeverendingColorReward::generateActionClips() {
    return std::shared_ptr<std::vector<std::shared_ptr<ActionClip> >>(new vector<std::shared_ptr<ActionClip>>(actionClips->begin(), actionClips->end()));
}

std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>> NeverendingColorReward::generatePerceptClips() {
    return std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>>(new vector<std::shared_ptr<PerceptClip>>(perceptClips->begin(), perceptClips->end()));
}

int NeverendingColorReward::getDimensionality() {
    return numberOfCategories + 1;
}
