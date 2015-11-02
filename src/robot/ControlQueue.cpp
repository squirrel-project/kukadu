#include "ControlQueue.h"

#include "../types/KukaduTypes.h"

using namespace std;
using namespace arma;

KUKADU_SHARED_PTR<kukadu_thread> ControlQueue::startQueueThread() {
    thr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&ControlQueue::run, this));
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
