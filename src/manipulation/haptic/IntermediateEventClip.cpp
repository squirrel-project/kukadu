#include "IntermediateEventClip.h"

#include <cstdio>
#include <iostream>

using namespace std;

IntermediateEventClip::IntermediateEventClip(std::shared_ptr<SensingController> sensingEvent,
                                             int level, std::shared_ptr<std::mt19937> generator, std::shared_ptr<std::vector<int>> clipValues, int immunity)
    : Clip(level, generator, clipValues, immunity) {

    this->caption = sensingEvent->getCaption();
    this->sensingEvent = sensingEvent;

}

std::pair<int, std::shared_ptr<Clip>> IntermediateEventClip::jumpNextRandom() {

    cout << "(IntermediateEventClip) perform sensing action " << sensingEvent->getCaption() << endl;

    pair<int, shared_ptr<Clip>> retVal;
    retVal.first = sensingEvent->performClassification();
    retVal.second = getSubClipByIdx(retVal.first);
    return retVal;

}
