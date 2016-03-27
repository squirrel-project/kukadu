#include <kukadu/types/controllerresult.hpp>

using namespace std;

namespace kukadu {

    ControllerResult::ControllerResult(arma::vec t, std::vector<arma::vec> ys) {
        construct(t, ys, true, false, vector<int>());
    }

    ControllerResult::ControllerResult(arma::vec t, std::vector<arma::vec> ys, bool success, bool bored, std::vector<int> walkedPath) {
        construct(t, ys, success, bored, walkedPath);
    }

    arma::vec ControllerResult::getTimes() {
        return t;
    }

    std::vector<arma::vec> ControllerResult::getYs() {
        return y;
    }

    void ControllerResult::construct(arma::vec t, std::vector<arma::vec> ys, bool success, bool bored, std::vector<int> walkedPath) {
        this->t = t;
        this->y = ys;
        this->bored = bored;
        this->success = success;
        this->walkedPath = walkedPath;
    }

    void ControllerResult::setSuccess(bool success) {
        this->success = success;
    }

    bool ControllerResult::getSuccess() {
        return success;
    }

    bool ControllerResult::wasBored() {
        return bored;
    }

    std::vector<int> ControllerResult::getWalkedPath() {
        return walkedPath;
    }

}
