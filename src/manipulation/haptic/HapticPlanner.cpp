#include "HapticPlanner.h"

HapticPlanner::HapticPlanner() {

}

void HapticPlanner::addSensingController(std::string name, std::shared_ptr<SensingController> sensingController) {

    sensingNames.push_back(name);
    sensingControllers.push_back(sensingController);

}

void HapticPlanner::addPreparationController(std::string name, std::shared_ptr<Controller> prepController) {

    preparationNames.push_back(name);
    preparationControllers.push_back(prepController);

}

void HapticPlanner::addComplexController(std::string name, std::shared_ptr<Controller> complexController) {

    complexControllerNames.push_back(name);
    complexControllers.push_back(complexController);

}
