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

public:

    virtual std::shared_ptr<ControllerResult> performAction() = 0;

};



#endif // CONTROLLER_H
