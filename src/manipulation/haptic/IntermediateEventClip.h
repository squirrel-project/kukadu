#ifndef INTERMEDIATEEVENTCLIP_H
#define INTERMEDIATEEVENTCLIP_H

#include "../SensingController.hpp"
#include "../../learning/projective_simulation/core/clip.h"

#include <cstdio>
#include <string>
#include <vector>
#include <memory>

class IntermediateEventClip : public Clip {

private:

    int hapticMode;

    std::string caption;

    std::shared_ptr<SensingController> sensingEvent;

public:

    IntermediateEventClip(std::shared_ptr<SensingController> sensingEvent,
                          int level, std::shared_ptr<std::mt19937> generator, std::shared_ptr<std::vector<int>> clipValues, int immunity);

    std::string toString() const;
    std::pair<int, std::shared_ptr<Clip>> jumpNextRandom();

    std::shared_ptr<SensingController> getSensingController();

};



#endif // INTERMEDIATEEVENTCLIP_H
