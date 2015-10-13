#include "manualreward.h"

#include <sstream>

using namespace std;

ManualReward::ManualReward(shared_ptr<std::mt19937> generator, int numberOfActions, int numberOfPercepts, bool collectPrevRewards, double stdReward) : Reward(generator, collectPrevRewards) {

    this->numberOfActions = numberOfActions;
    this->numberOfPercepts = numberOfPercepts;
    this->nextPerceptId = 0;
    this->stdReward = stdReward;

    perceptClips = shared_ptr<vector<shared_ptr<PerceptClip>>>(new vector<shared_ptr<PerceptClip>>());
    for(int i = 0; i < numberOfPercepts; ++i) {
        stringstream s;
        s << i;
        std::shared_ptr<std::vector<int>> clipDimensionValues = std::shared_ptr<std::vector<int>>(new vector<int>());
        clipDimensionValues->push_back(i);
        perceptClips->push_back(std::shared_ptr<PerceptClip>(new PerceptClip(i, s.str(), generator, clipDimensionValues, PS_DEFAULT_IMMUNITY)));
    }

    actionClips = shared_ptr<vector<shared_ptr<ActionClip>>>(new vector<shared_ptr<ActionClip>>());
    for(int i = 0; i < numberOfActions; ++i) {
        stringstream s;
        s << i;
        std::shared_ptr<std::vector<int>> clipDimensionValues = std::shared_ptr<std::vector<int>>(new vector<int>());
        clipDimensionValues->push_back(i);
        actionClips->push_back(std::shared_ptr<ActionClip>(new ActionClip(i, 1, s.str(), generator)));
    }

}

ManualReward::~ManualReward() {

}

void ManualReward::setNextPerceptId(int nextId) {
    this->nextPerceptId = nextId;
}

int ManualReward::getDimensionality() {

    return 1;

}

std::shared_ptr<PerceptClip> ManualReward::generateNextPerceptClip(int immunity) {

    return perceptClips->at(nextPerceptId);

}

double ManualReward::computeRewardInternal(std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction) {

    int worked = 0;
    double retReward = 0.0;
    cout << "selected percept " << *providedPercept << " resulted in action " << *takenAction << endl;
    cout << "was it the correction action (0 = no / 1 = yes)" << endl;
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

std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> ManualReward::generateActionClips() {
    return actionClips;
}

std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>> ManualReward::generatePerceptClips() {
    return perceptClips;
}
