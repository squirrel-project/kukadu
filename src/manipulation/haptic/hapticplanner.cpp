#include <kukadu/manipulation/haptic/hapticplanner.hpp>
#include <kukadu/manipulation/complexcontroller.hpp>

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

            string envModelPath = complexPath + "envmodels";
            preparePathString(envModelPath);

            string hapticPath = complexPath + "haptics";
            preparePathString(hapticPath);

            auto sensingCopy = copySensingControllers(sensingControllers, hapticPath);

            if(!fileExists(complexPath)) {

                // initialize haptic planner in general (load controllers and create ps)
                createDirectory(complexPath);
                castCompCont->setSensingControllers(sensingCopy);
                castCompCont->setPreparatoryControllers(preparatoryControllers);
                castCompCont->initialize();
                castCompCont->store(complexPath);

                createDirectory(envModelPath);

                // load environment model
                for(auto sens : sensingControllers) {
                    auto envModel = createEnvironmentModelForSensingAction(sens, preparatoryControllers);
                    environmentModels.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> >(sens->getCaption(), envModel));
                    envModel->storePS(envModelPath + sens->getCaption());
                }

            } else {

                castCompCont->load(complexPath, registeredSensingControllers, registeredPrepControllers);

                // load environment model
                for(auto sens : sensingCopy) {
                    auto envModel = KUKADU_SHARED_PTR<ProjectiveSimulator>(new ProjectiveSimulator(nullptr, generator, envModelPath + sens->getCaption()));
                    environmentModels.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> >(sens->getCaption(), envModel));
                }

            }

            registeredComplexControllers.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::Controller> >(contName, compCont));

        }

    }

    void HapticPlanner::pickAndPerformComplexSkill() {

        auto selectedId = pickComplexSkill();
        performComplexSkill(selectedId);

    }

    void HapticPlanner::performComplexSkill(std::string skillId) {

        auto complSkill = registeredComplexControllers[skillId];
        complSkill->performAction();

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

    KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> HapticPlanner::createEnvironmentModelForSensingAction(KUKADU_SHARED_PTR<kukadu::SensingController> sensingAction,
                                                std::vector<KUKADU_SHARED_PTR<kukadu::Controller> >& preparatoryActions) {

        int sensingCatCount = sensingAction->getSensingCatCount();
        int prepActionsCount = preparatoryActions.size();

        auto environmentPercepts = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<PerceptClip> > >(new vector<KUKADU_SHARED_PTR<PerceptClip> >());
        auto resultingStatePercepts = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<Clip> > >(new vector<KUKADU_SHARED_PTR<Clip> >());

        auto idVec = KUKADU_SHARED_PTR< vector<int> >(new vector<int>{0, 0});
        for(int i = 0; i < sensingCatCount; ++i) {
            stringstream s;
            s << "E" << i;
            resultingStatePercepts->push_back(KUKADU_SHARED_PTR<ActionClip>(new ActionClip(i, idVec->size(), s.str(), generator)));
        }

        for(int stateId = 0, overallId = sensingCatCount; stateId < sensingCatCount; ++stateId) {

            idVec->at(0) = stateId;

            for(int actId = 0; actId < prepActionsCount; ++actId, ++overallId) {

                idVec->at(1) = actId;

                stringstream s;
                s << "(E" << stateId << ",P" << actId << ")";
                auto vecCopy = KUKADU_SHARED_PTR<vector<int> >(new vector<int>(idVec->begin(), idVec->end()));
                auto newPercept = KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(overallId, s.str(), generator, vecCopy, INT_MAX));
                newPercept->setChildren(resultingStatePercepts);
                environmentPercepts->push_back(newPercept);

            }

        }

        auto retProjSim = KUKADU_SHARED_PTR<ProjectiveSimulator>(new ProjectiveSimulator(nullptr, generator, environmentPercepts, 0.0, ProjectiveSimulator::PS_USE_ORIGINAL, false));
        return retProjSim;

    }

    void HapticPlanner::performSkill(std::string skillIdx) {

        KUKADU_SHARED_PTR<kukadu::Controller> complCont = registeredComplexControllers[skillIdx];

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
