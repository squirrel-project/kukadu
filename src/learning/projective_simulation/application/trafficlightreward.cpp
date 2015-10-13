#include "trafficlightreward.h"

using namespace std;

TrafficLightReward::TrafficLightReward(std::shared_ptr<std::mt19937> generator, bool collectPrevRewards) : Reward(generator, collectPrevRewards) {

    currentT = 0;
    intDist = std::uniform_int_distribution<int>(0, 3);
    actionClips = std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>>(new vector<std::shared_ptr<ActionClip>>());
    actionClips->push_back(std::shared_ptr<ActionClip>(new ActionClip(0, 2, "drive", generator)));
    actionClips->push_back(std::shared_ptr<ActionClip>(new ActionClip(1, 2, "stop", generator)));

    perceptClips = std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>>(new vector<std::shared_ptr<PerceptClip>>());
    // (0, 0) = (green, left)

    perceptClips->push_back(std::shared_ptr<PerceptClip>(new PerceptClip(TRAFFICLIGHTREWARD_GREEN_RIGHT, "(green, left)", generator, std::shared_ptr<vector<int>>(new vector<int>({0, 0})), PS_DEFAULT_IMMUNITY)));
    perceptClips->push_back(std::shared_ptr<PerceptClip>(new PerceptClip(TRAFFICLIGHTREWARD_GREEN_LEFT, "(green, right)", generator, std::shared_ptr<vector<int>>(new vector<int>({0, 1})), PS_DEFAULT_IMMUNITY)));
    perceptClips->push_back(std::shared_ptr<PerceptClip>(new PerceptClip(TRAFFICLIGHTREWARD_RED_RIGHT, "(red, left)", generator, std::shared_ptr<vector<int>>(new vector<int>({1, 0})), PS_DEFAULT_IMMUNITY)));
    perceptClips->push_back(std::shared_ptr<PerceptClip>(new PerceptClip(TRAFFICLIGHTREWARD_RED_LEFT, "(red, right)", generator, std::shared_ptr<vector<int>>(new vector<int>({1, 1})), PS_DEFAULT_IMMUNITY)));

}

std::shared_ptr<PerceptClip> TrafficLightReward::generateNextPerceptClip(int immunity) {

    int idx = intDist(*generator);
    return perceptClips->at(idx);

}

double TrafficLightReward::computeRewardInternal(std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction) {

    double reward;
    ++currentT;

    int perceptId = providedPercept->getPerceptId();
    int actionId = takenAction->getActionId();

    // ignore arrow (green -> drive, red -> stop)
    if(currentT <= 1000) {
        if(
                (perceptId == TRAFFICLIGHTREWARD_GREEN_RIGHT && actionId == TRAFFICLIGHTREWARD_DRIVE) ||
                (perceptId == TRAFFICLIGHTREWARD_GREEN_LEFT && actionId == TRAFFICLIGHTREWARD_DRIVE) ||
                (perceptId == TRAFFICLIGHTREWARD_RED_RIGHT && actionId == TRAFFICLIGHTREWARD_STOP) ||
                (perceptId == TRAFFICLIGHTREWARD_RED_LEFT && actionId == TRAFFICLIGHTREWARD_STOP)
                ) {
            reward = 1.0;
        } else {
            reward = 0.0;
        }
    }
    // doing the opposite to the first case (green -> stop, red -> drive)
    else if(currentT <= 2000) {
        if(
                (perceptId == TRAFFICLIGHTREWARD_GREEN_RIGHT && actionId == TRAFFICLIGHTREWARD_DRIVE) ||
                (perceptId == TRAFFICLIGHTREWARD_GREEN_LEFT && actionId == TRAFFICLIGHTREWARD_DRIVE) ||
                (perceptId == TRAFFICLIGHTREWARD_RED_RIGHT && actionId == TRAFFICLIGHTREWARD_STOP) ||
                (perceptId == TRAFFICLIGHTREWARD_RED_LEFT && actionId == TRAFFICLIGHTREWARD_STOP)
                ) {
            reward = 0.0;
        } else {
            reward = 1.0;
        }
    }
    // ignore color (left -> drive, right -> stop)
    else if(currentT <= 3000) {
        if(
                (perceptId == TRAFFICLIGHTREWARD_GREEN_RIGHT && actionId == TRAFFICLIGHTREWARD_STOP) ||
                (perceptId == TRAFFICLIGHTREWARD_GREEN_LEFT && actionId == TRAFFICLIGHTREWARD_DRIVE) ||
                (perceptId == TRAFFICLIGHTREWARD_RED_RIGHT && actionId == TRAFFICLIGHTREWARD_STOP) ||
                (perceptId == TRAFFICLIGHTREWARD_RED_LEFT && actionId == TRAFFICLIGHTREWARD_DRIVE)
                ) {
            reward = 1.0;
        } else {
            reward = 0.0;
        }
    }
    // always reward drive
    else {
        if(actionId == TRAFFICLIGHTREWARD_DRIVE)
            reward = 1.0;
        else
            reward = 0.0;
    }

    return reward;

}

std::shared_ptr<std::vector<std::shared_ptr<ActionClip> > > TrafficLightReward::generateActionClips() {
    return actionClips;
}

std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>> TrafficLightReward::generatePerceptClips() {
    return std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>>(new std::vector<std::shared_ptr<PerceptClip>>());
}

int TrafficLightReward::getDimensionality() {
    return 2;
}
