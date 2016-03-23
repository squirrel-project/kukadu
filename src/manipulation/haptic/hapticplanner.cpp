#include <kukadu/manipulation/haptic/hapticplanner.hpp>
#include <kukadu/manipulation/complexcontroller.hpp>

using namespace std;

namespace kukadu {

    HapticPlanner::HapticPlanner(std::string skillDatabase,
                                 std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > sensingControllers,
                                 std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > preparatoryControllers,
                                 std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > complexControllers) {

        preparePathString(skillDatabase);

        this->skillDatabase = skillDatabase;

        for(auto sensCont : sensingControllers)
            registeredSensingControllers.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::SensingController> >(sensCont->getCaption(), sensCont));

        for(auto prepCont : preparatoryControllers)
            registeredPrepControllers.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::Controller> >(prepCont->getCaption(), prepCont));

        for(auto compCont : complexControllers) {

            KUKADU_SHARED_PTR<kukadu::ComplexController> castCompCont = KUKADU_DYNAMIC_POINTER_CAST<kukadu::ComplexController>(compCont);
            string contName = compCont->getCaption();
            string complexPath = skillDatabase + contName;
            preparePathString(complexPath);

            if(!fileExists(complexPath)) {

                createDirectory(complexPath);
                castCompCont->setSensingControllers(sensingControllers);
                castCompCont->setPreparatoryControllers(preparatoryControllers);
                castCompCont->initialize();
                castCompCont->store(complexPath);

            } else {

                castCompCont->load(complexPath, registeredSensingControllers, registeredPrepControllers);

            }

            registeredComplexControllers.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::Controller> >(contName, compCont));

        }

    }

    std::string HapticPlanner::pickComplexController() {



    }

    void HapticPlanner::performSkill(std::string skillIdx) {

        KUKADU_SHARED_PTR<kukadu::Controller> complCont = registeredComplexControllers[skillIdx];

    }

}
