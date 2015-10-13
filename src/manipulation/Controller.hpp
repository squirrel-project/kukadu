#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cstdio>
#include <iostream>
#include <fstream>
#include <thread>
#include <string>
#include <vector>
#include <wordexp.h>
#include <memory>

#include "ControllerResult.hpp"

class Controller {

private:

    bool simulation;
    std::string caption;

protected:

    bool isShutUp;

    // is called by set simulation mode
    virtual void setSimulationModeInChain(bool simulationMode) {}

public:

    Controller(std::string caption);

    std::string getCaption();
    virtual std::shared_ptr<ControllerResult> performAction() = 0;

    void setSimulationMode(bool simulationMode);
    bool getSimulationMode();

    void shutUp();
    void startTalking();

};



#endif // CONTROLLER_H
