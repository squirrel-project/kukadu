#ifndef INVASIONGAMEREWARD_H
#define INVASIONGAMEREWARD_H

#include <random>
#include "../core/reward.h"

#define INVASIONGAMEREWARD_H_LEFT_PERCEPT 0
#define INVASIONGAMEREWARD_H_RIGHT_PERCEPT 1

#define INVASIONGAMEREWARD_H_LEFT_ACTION 0
#define INVASIONGAMEREWARD_H_RIGHT_ACTION 1

#define INVASIONGAMEREWARD_REW 1.0

class InvasionGameReward : public Reward {

private:

    int currentTimeStep;
    std::uniform_int_distribution<int> intDist;
    std::shared_ptr<std::vector<std::shared_ptr<PerceptClip> >> perceptClips;

protected:

    double computeRewardInternal(std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction);

public:

    InvasionGameReward(std::shared_ptr<std::mt19937> generator, bool collectPrevRewards);

    int getDimensionality();
    std::shared_ptr<PerceptClip> generateNextPerceptClip(int immunity);

    std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> generateActionClips();
    std::shared_ptr<std::vector<std::shared_ptr<PerceptClip> > > generatePerceptClips();

};

#endif // INVASIONGAMEREWARD_H
