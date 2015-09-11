#include "ControlQueue.h"

using namespace std;
using namespace arma;

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

double ControlQueue::getAbsoluteCartForce() {

    mes_result m = getCurrentCartesianFrcTrq();
    vec forces = m.joints.subvec(0, 2);
    vec prod = forces.t() * forces;
    return sqrt(prod(0));

}
