#include "ControllerActionClip.h"

using namespace std;

ControllerActionClip::ControllerActionClip(int actionId, std::shared_ptr<Controller> actionController,
                      std::shared_ptr<std::mt19937> generator) : ActionClip(actionId, 1, actionController->getCaption(), generator) {
    this->actionController = actionController;
}

void ControllerActionClip::performAction() {

    int executeIt = 0;
    cout << "(ControllerActionClip) selected preparation action is \"" << actionController->getCaption() << "\"; want to execute it? (0 = no / 1 = yes)" << endl;
    cin >> executeIt;

    if(executeIt == 1) {
        actionController->performAction();
    } else {
        cout << "(ControllerActionClip) you decided not to perform the action; continue" << endl;
    }
}
