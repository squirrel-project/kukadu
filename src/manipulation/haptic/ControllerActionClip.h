#ifndef KUKADU_CONTROLLERACTIONCLIP_H
#define KUKADU_CONTROLLERACTIONCLIP_H

#include "../Controller.hpp"
#include "../../learning/projective_simulation/core/actionclip.h"

#include <cstdio>
#include <string>
#include <vector>

class ControllerActionClip : public ActionClip {

private:

    std::string caption;

    KUKADU_SHARED_PTR<Controller> actionController;

public:

    ControllerActionClip(int actionId, KUKADU_SHARED_PTR<Controller> actionController,
                          KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator);

    void performAction();

};



#endif // CONTROLLERACTIONCLIP_H
