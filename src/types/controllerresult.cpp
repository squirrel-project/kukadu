#include <kukadu/types/controllerresult.hpp>

using namespace std;

namespace kukadu {

    ControllerResult::ControllerResult(arma::vec t, std::vector<arma::vec> ys, bool success) {
        this->t = t;
        this->y = ys;
        this->success = success;
    }

    HapticControllerResult::HapticControllerResult(arma::vec t, std::vector<arma::vec> ys, bool success, bool bored, std::vector<int> walkedPath,
                                                   KUKADU_SHARED_PTR<std::tuple<double, KUKADU_SHARED_PTR<kukadu::Clip>, std::vector<KUKADU_SHARED_PTR<kukadu::Clip> > > > environmentTransition)
        : ControllerResult(t, ys, success) {

        this->bored = bored;
        this->walkedPath = walkedPath;
        this->environmentTransition = environmentTransition;

    }

    arma::vec ControllerResult::getTimes() {
        return t;
    }

    std::vector<arma::vec> ControllerResult::getYs() {
        return y;
    }

    void ControllerResult::setSuccess(bool success) {
        this->success = success;
    }

    bool ControllerResult::getSuccess() {
        return success;
    }

    bool HapticControllerResult::wasBored() {
        return bored;
    }

    std::vector<int> HapticControllerResult::getWalkedPath() {
        return walkedPath;
    }

    void HapticControllerResult::setEntropyMeanAndVariance(std::vector<std::pair<double, double> > meanAndVar) {
        this->meanAndVar = meanAndVar;
    }

    std::vector<std::pair<double, double> > HapticControllerResult::getMeanAndVar() {
        return meanAndVar;
    }

}
