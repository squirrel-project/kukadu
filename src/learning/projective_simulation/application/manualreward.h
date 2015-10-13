#ifndef MANUALREWARD_H
#define MANUALREWARD_H

#include <random>
#include "../core/reward.h"

class ManualReward : public Reward {

private:

    std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> actionClips;
    std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>> perceptClips;

    int nextPerceptId;
    int numberOfActions;
    int numberOfPercepts;

    double stdReward;

protected:

    double computeRewardInternal(std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction);

public:

    ManualReward(std::shared_ptr<std::mt19937> generator, int numberOfActions, int numberOfPercepts, bool collectPrevRewards, double stdReward);
    ~ManualReward();

    void setNextPerceptId(int nextId);

    int getDimensionality();
    std::shared_ptr<PerceptClip> generateNextPerceptClip(int immunity);

    std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> generateActionClips();
    std::shared_ptr<std::vector<std::shared_ptr<PerceptClip> >> generatePerceptClips();

};

#endif // MANUALREWARD_H
