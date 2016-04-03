#include <kukadu/manipulation/haptic/hapticplanner.hpp>
#include <kukadu/manipulation/complexcontroller.hpp>
#include <kukadu/types/kukadutypes.hpp>

using namespace std;

namespace kukadu {

    HapticPlanner::HapticPlanner(std::string skillDatabase,
                                 std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > sensingControllers,
                                 std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > preparatoryControllers,
                                 std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > complexControllers,
                                 KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator) : Reward(generator, false) {

        this->generator = generator;

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

            string hapticPath = complexPath + "haptics";
            preparePathString(hapticPath);

            auto sensingCopy = copySensingControllers(sensingControllers, hapticPath);

            if(!fileExists(complexPath))
                // initialize haptic planner in general (load controllers and create ps)
                createDirectory(complexPath);

            if(!fileExists(complexPath + "composition")) {

                castCompCont->setSensingControllers(sensingCopy);
                castCompCont->setPreparatoryControllers(preparatoryControllers);
                castCompCont->initialize();
                castCompCont->store(complexPath);

            } else {

                castCompCont->load(complexPath, registeredSensingControllers, registeredPrepControllers);

            }

            registeredComplexControllers.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::Controller> >(contName, compCont));

        }

    }

    void HapticPlanner::pickAndPerformComplexSkill() {

        auto selectedId = pickComplexSkill();
        performComplexSkill(selectedId);

    }

    KUKADU_SHARED_PTR<kukadu::HapticControllerResult> HapticPlanner::performComplexSkill(std::string skillId) {

        auto complSkill = registeredComplexControllers[skillId];
        return KUKADU_DYNAMIC_POINTER_CAST<HapticControllerResult>(complSkill->performAction());

    }

    std::string HapticPlanner::pickComplexSkill() {

        int selection = 0;
        map<int, string> keyMap;
        cout << "select a complex skill:" << endl << "======================================" << endl;
        int i = 0;
        for(auto comp : registeredComplexControllers) {
            cout << "(" << i << ") " << comp.second->getCaption() << endl;
            keyMap.insert(pair<int, string>(i, comp.second->getCaption()));
        }
        cout << endl << "selection: ";
        cin >> selection;
        return keyMap[i];

    }

    KUKADU_SHARED_PTR<PerceptClip> HapticPlanner::generateNextPerceptClip(int immunity) {
        throw KukaduException("generateNextPerceptClip not implemented yet");
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > HapticPlanner::generateActionClips() {
        throw KukaduException("generateActionClips not implemented yet");
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > HapticPlanner::generatePerceptClips() {
        throw KukaduException("generatePerceptClips not implemented yet");
    }

    double HapticPlanner::computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction) {
        throw KukaduException("compouteRewardInternal not implemented yet");
    }

    int HapticPlanner::getDimensionality() {
        throw KukaduException("getDimensionlity not implemented yet");
    }

    std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > HapticPlanner::copySensingControllers(std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > controllers,
                                                                                      std::string newBasePath) {

        vector<KUKADU_SHARED_PTR<kukadu::SensingController> > retVec;
        for(auto sens : controllers) {
            auto copiedSens = sens->clone();
            copiedSens->setDatabasePath(newBasePath + sens->getCaption() + "/");
            copiedSens->createDataBase();
            retVec.push_back(copiedSens);
        }
        return retVec;

    }

}
