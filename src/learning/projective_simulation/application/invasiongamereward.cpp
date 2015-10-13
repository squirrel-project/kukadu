#include "invasiongamereward.h"

using namespace std;

InvasionGameReward::InvasionGameReward(std::shared_ptr<std::mt19937> generator, bool collectPrevRewards) : Reward(generator, collectPrevRewards) {
    currentTimeStep = 0;
    intDist = std::uniform_int_distribution<int>(0, 1);
}

double InvasionGameReward::computeRewardInternal(std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction) {

    int perceptId = providedPercept->getPerceptId();
    int actionId = takenAction->getActionId();
    if(++currentTimeStep < 250)
        return INVASIONGAMEREWARD_REW * (1 - abs(perceptId - actionId));
    else
        return INVASIONGAMEREWARD_REW * (abs(perceptId - actionId));

}

std::shared_ptr<PerceptClip> InvasionGameReward::generateNextPerceptClip(int immunity) {
    return perceptClips->at(intDist(*generator));
}

std::shared_ptr<std::vector<std::shared_ptr<ActionClip> > > InvasionGameReward::generateActionClips() {
    std::shared_ptr<vector<std::shared_ptr<ActionClip>>> actionClips = std::shared_ptr<vector<std::shared_ptr<ActionClip>>>(new vector<std::shared_ptr<ActionClip>>());
    actionClips->push_back(std::shared_ptr<ActionClip>(new ActionClip(INVASIONGAMEREWARD_H_LEFT_ACTION, 1, "left action", generator)));
    actionClips->push_back(std::shared_ptr<ActionClip>(new ActionClip(INVASIONGAMEREWARD_H_RIGHT_ACTION, 1, "right action", generator)));
    return actionClips;
}

std::shared_ptr<std::vector<std::shared_ptr<PerceptClip> >> InvasionGameReward::generatePerceptClips() {
    std::shared_ptr<vector<std::shared_ptr<PerceptClip>>> perceptClips = std::shared_ptr<vector<std::shared_ptr<PerceptClip>>>(new vector<std::shared_ptr<PerceptClip>>());
    perceptClips->push_back(std::shared_ptr<PerceptClip>(new PerceptClip(INVASIONGAMEREWARD_H_LEFT_PERCEPT, "left sign", generator, std::shared_ptr<vector<int>>(new vector<int>({INVASIONGAMEREWARD_H_LEFT_PERCEPT})), PS_DEFAULT_IMMUNITY)));
    perceptClips->push_back(std::shared_ptr<PerceptClip>(new PerceptClip(INVASIONGAMEREWARD_H_RIGHT_PERCEPT, "right sign", generator, std::shared_ptr<vector<int>>(new vector<int>({INVASIONGAMEREWARD_H_RIGHT_PERCEPT})), PS_DEFAULT_IMMUNITY)));
    return std::shared_ptr<std::vector<std::shared_ptr<PerceptClip> >>(this->perceptClips = perceptClips);
}

int InvasionGameReward::getDimensionality() {
    return 1;
}
