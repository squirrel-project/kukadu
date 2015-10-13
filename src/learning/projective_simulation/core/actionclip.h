#ifndef ACTIONCLIP_H
#define ACTIONCLIP_H

#include <string>
#include <random>
#include "clip.h"

class ActionClip : public Clip {

private:

    int actionId;
    std::string label;

public:

    ActionClip(int actionId, int perceptDimensionality, std::string label, std::shared_ptr<std::mt19937> generator);

    int getActionId();

    std::string getLabel();
    std::string toString() const;

    std::pair<int, std::shared_ptr<Clip>> jumpNextRandom();

};

#endif // ACTIONCLIP_H
