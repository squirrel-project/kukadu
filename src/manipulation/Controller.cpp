#include "Controller.hpp"

Controller::Controller(std::string caption) {

    isShutUp = true;

    this->caption = caption;
    this->simulation = false;
}

std::string Controller::getCaption() {
    return caption;
}

void Controller::setSimulationMode(bool simulationMode) {
    simulation = simulationMode;
    setSimulationModeInChain(simulationMode);
}

bool Controller::getSimulationMode() {
    return simulation;
}

void Controller::shutUp() {
    isShutUp = true;
}

void Controller::startTalking() {
    isShutUp = false;
}
