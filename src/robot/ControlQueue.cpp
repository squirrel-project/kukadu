#include "ControlQueue.h"

using namespace std;

std::shared_ptr<std::thread> ControlQueue::startQueueThread() {
    thr = std::shared_ptr<std::thread>(new std::thread(&ControlQueue::run, this));
    while(!this->isInitialized());
    return thr;
}

ControlQueue::ControlQueue(int degOfFreedom) {
	this->degOfFreedom = degOfFreedom;
}

int ControlQueue::getMovementDegreesOfFreedom() {
	return degOfFreedom;
}
