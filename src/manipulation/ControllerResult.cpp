#include "ControllerResult.hpp"


ControllerResult::ControllerResult(arma::vec t, std::vector<arma::vec> ys) {
    this->t = t;
    this->y = ys;
}

arma::vec ControllerResult::getTimes() {
    return t;
}

std::vector<arma::vec> ControllerResult::getYs() {
    return y;
}
