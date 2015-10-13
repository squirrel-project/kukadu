#ifndef NEVERENDINGCOLORREWARD_H
#define NEVERENDINGCOLORREWARD_H

#include <random>
#include "../core/reward.h"

#define NEVERENDINGCOLORREWARD_LEFT 0
#define NEVERENDINGCOLORREWARD_RIGHT 1

// used for experiment with ranking
//#define NEVERENDINGCOLORREWARD_SUCCESSFUL_REWARD 1000

// used for experiment without ranking to get faster convergence
#define NEVERENDINGCOLORREWARD_SUCCESSFUL_REWARD 10000

class NeverendingColorReward : public Reward {

private:

    int currentId;
    int currentT;
    int numberOfActions;
    int numberOfCategories;
    std::uniform_int_distribution<int> intDist;
    std::vector<std::shared_ptr<ActionClip>>* actionClips;
    std::vector<std::shared_ptr<PerceptClip>>* perceptClips;

protected:

    double computeRewardInternal(std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction);

public:

    NeverendingColorReward(std::shared_ptr<std::mt19937> generator, int numberOfActions, int numberOfCategories, bool collectPrevRewards);
    ~NeverendingColorReward();

    int getDimensionality();
    std::shared_ptr<PerceptClip> generateNextPerceptClip(int immunity);

    std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> generateActionClips();
    std::shared_ptr<std::vector<std::shared_ptr<PerceptClip> >> generatePerceptClips();

};

#endif // NEVERENDINGCOLORREWARD_H
