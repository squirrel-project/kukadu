#include "actionclip.h"

using namespace std;

ActionClip::ActionClip(int actionId, int perceptDimensionality, std::string label, std::shared_ptr<std::mt19937> generator) : Clip(CLIP_H_LEVEL_FINAL, generator, std::shared_ptr<vector<int>>(new vector<int>()), INT_MAX) {
    this->actionId = actionId;
    this->label = label;
    std::shared_ptr<vector<int>> dummyVals = std::shared_ptr<vector<int>>(new vector<int>());
    for(int i = 0; i < perceptDimensionality; ++i)
        dummyVals->push_back(-actionId - 1);
    setClipDimensionValues(dummyVals);
}

int ActionClip::getActionId() {
    return actionId;
}

std::string ActionClip::getLabel() {
    return label;
}

std::pair<int, std::shared_ptr<Clip> > ActionClip::jumpNextRandom() {
    return pair<int, std::shared_ptr<Clip>>(0, shared_from_this());
}

std::string ActionClip::toString() const {
    return label;
}
