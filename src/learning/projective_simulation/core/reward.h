#ifndef REWARD_H
#define REWARD_H

#include "actionclip.h"
#include "perceptclip.h"

#include <memory>
#include <random>
#include <vector>

class Reward {

private:

    bool collectPrevRewards;
    std::vector<double> previousRewards;

protected:

    std::shared_ptr<std::mt19937> generator;

    virtual double computeRewardInternal(std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction) = 0;

public:

    Reward(std::shared_ptr<std::mt19937> generator, bool collectPrevRewards);

    std::vector<double> getPreviousRewards();
    double computeReward(std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction);

    virtual int getDimensionality() = 0;

    // same percept must always have same id
    virtual std::shared_ptr<PerceptClip> generateNextPerceptClip(int immunity) = 0;

    virtual std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> generateActionClips() = 0;
    virtual std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>> generatePerceptClips() = 0;

};

#endif // REWARD_H
