#ifndef TRAFFICLIGHTREWARD_H
#define TRAFFICLIGHTREWARD_H

#include <random>
#include "../core/reward.h"

#define TRAFFICLIGHTREWARD_GREEN_RIGHT 0 * 2 + 0
#define TRAFFICLIGHTREWARD_GREEN_LEFT 0 * 2 + 1
#define TRAFFICLIGHTREWARD_RED_RIGHT 1 * 2 + 0
#define TRAFFICLIGHTREWARD_RED_LEFT 1 * 2 + 1

#define TRAFFICLIGHTREWARD_DRIVE 0
#define TRAFFICLIGHTREWARD_STOP 1

class TrafficLightReward : public Reward {

private:

    int currentT;
    std::uniform_int_distribution<int> intDist;
    std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> actionClips;
    std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>> perceptClips;

protected:

    double computeRewardInternal(std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction);

public:

    TrafficLightReward(std::shared_ptr<std::mt19937> generator, bool collectPrevRewards);

    int getDimensionality();
    std::shared_ptr<PerceptClip> generateNextPerceptClip(int immunity);

    std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> generateActionClips();
    std::shared_ptr<std::vector<std::shared_ptr<PerceptClip> >> generatePerceptClips();

};

#endif // TRAFFICLIGHTREWARD_H
