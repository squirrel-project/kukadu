#ifndef CONTROLLERACTIONCLIP_H
#define CONTROLLERACTIONCLIP_H

#include "../Controller.hpp"
#include "../../learning/projective_simulation/core/actionclip.h"

#include <cstdio>
#include <string>
#include <vector>
#include <memory>

class ControllerActionClip : public ActionClip {

private:

    std::string caption;

    std::shared_ptr<Controller> actionController;

public:

    ControllerActionClip(int actionId, std::shared_ptr<Controller> actionController,
                          std::shared_ptr<std::mt19937> generator);

    void performAction();

};



#endif // CONTROLLERACTIONCLIP_H
