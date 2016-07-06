#include <kukadu/manipulation/concatcontroller.hpp>

using namespace std;

namespace kukadu {

    std::string ConcatController::generateLabelFromControllers(std::vector<KUKADU_SHARED_PTR<kukadu::Controller> >& controllers) {

        string retLabel = "";
        bool first = true;
        for(auto cont : controllers) {
            if(first)
                first = false;
            else retLabel += " ";
            retLabel += cont->getCaption();
        }
        return retLabel;

    }

    int computeSimulationFailingProbability(std::vector<KUKADU_SHARED_PTR<kukadu::Controller> >& controllers) {

        int totalSuccProb = 1.0;
        for(auto cont : controllers)
            totalSuccProb *= 1.0 - cont->getSimFailingProb();
        return 1.0 - totalSuccProb;

    }

    ConcatController::ConcatController(std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > controllers)
        : Controller(generateLabelFromControllers(controllers), computeSimulationFailingProbability(controllers)) {
        this->controllers = controllers;
    }

    bool ConcatController::getSimulationMode() {
        for(auto cont : controllers)
            if(!cont->getSimulationMode())
                return false;
        return true;
    }

    bool ConcatController::requiresGrasp() {

        return controllers.front()->producesGrasp();

    }

    bool ConcatController::producesGrasp() {

        return controllers.back()->producesGrasp();

    }

    KUKADU_SHARED_PTR<ControllerResult> ConcatController::performAction() {

        KUKADU_SHARED_PTR<ControllerResult> lastResult;
        // execute all controllers
        for(auto& cont : controllers)
            lastResult = cont->performAction();

        // dummy for now takes only the last result
        return lastResult;

    }

}
