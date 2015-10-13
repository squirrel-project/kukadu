#include "reward.h"

using namespace std;

Reward::Reward(shared_ptr<std::mt19937> generator, bool collectPrevRewards) {
    this->generator = generator;
    this->collectPrevRewards = collectPrevRewards;
}

double Reward::computeReward(std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction) {
    double retrievedReward = computeRewardInternal(providedPercept, takenAction);
    if(collectPrevRewards)
        previousRewards.push_back(retrievedReward);
    return retrievedReward;
}

std::vector<double> Reward::getPreviousRewards() {
    return previousRewards;
}
