#include "Controller.hpp"

Controller::Controller(std::string caption) {
    this->caption = caption;
}

std::string Controller::getCaption() {
    return caption;
}
