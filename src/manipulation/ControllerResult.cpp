#include "ControllerResult.hpp"

ControllerResult::ControllerResult(arma::vec t, std::vector<arma::vec> ys) {
    construct(t, ys, true);
}

ControllerResult::ControllerResult(arma::vec t, std::vector<arma::vec> ys, bool success) {
    construct(t, ys, success);
}

arma::vec ControllerResult::getTimes() {
    return t;
}

std::vector<arma::vec> ControllerResult::getYs() {
    return y;
}

void ControllerResult::construct(arma::vec t, std::vector<arma::vec> ys, bool success) {
    this->t = t;
    this->y = ys;
    this->success = success;
}

void ControllerResult::setSuccess(bool success) {
    this->success = success;
}

bool ControllerResult::getSuccess() {
    return success;
}
