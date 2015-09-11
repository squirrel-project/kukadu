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

    std::string caption;

public:

    Controller(std::string caption);

    std::string getCaption();
    virtual std::shared_ptr<ControllerResult> performAction() = 0;

};



#endif // CONTROLLER_H
