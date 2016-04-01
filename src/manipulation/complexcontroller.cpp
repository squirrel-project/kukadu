#include <tuple>
#include <armadillo>
#include <kukadu/manipulation/controller.hpp>
#include <kukadu/manipulation/complexcontroller.hpp>
#include <kukadu/manipulation/haptic/controlleractionclip.hpp>
#include <kukadu/manipulation/haptic/intermediateeventclip.hpp>
#include <kukadu/learning/projective_simulation/core/clip.hpp>
#include <kukadu/learning/projective_simulation/core/perceptclip.hpp>

using namespace std;
using namespace arma;

namespace kukadu {

#ifdef USEBOOST
    const std::string ComplexController::FILE_SENSING_PREFIX = "***sensing controllers:";
    const std::string ComplexController::FILE_PREP_PREFIX = "***preparatory controllers:";
    const std::string ComplexController::FILE_END_PREFIX = "***end";
#endif

    ComplexController::ComplexController(std::string caption, std::string storePath,
                                         bool storeReward, double senseStretch, double boredom, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator,
                                         int stdReward, double punishReward, double gamma, int stdPrepWeight, bool collectPrevRewards,
                                         int simulationFailingProbability, int maxEnvPathLength, double pathLengthCost)
        : Controller(caption, simulationFailingProbability), Reward(generator, collectPrevRewards) {

        projSim.reset();
        rewardHistoryStream.reset();

        preparePathString(storePath);

        this->gamma = gamma;
        this->gen = generator;
        this->boredom = boredom;
        this->currentIterationNum = 0;
        this->storeReward = storeReward;
        this->senseStretch = senseStretch;
        this->colPrevRewards = collectPrevRewards;
        this->rewardHistoryPath = storePath + "rewards/";

        this->pathLengthCost = pathLengthCost;
        this->maxEnvPathLength = maxEnvPathLength;

        if(!fileExists(rewardHistoryPath))
            createDirectory(rewardHistoryPath);

        this->stdReward = stdReward;
        this->storePath = storePath;
        this->punishReward = punishReward;
        this->stdPrepWeight = stdPrepWeight;

        vector<int> distributionWeights; distributionWeights.push_back(100 - simulationFailingProbability); distributionWeights.push_back(simulationFailingProbability);
        simSuccDist = KUKADU_DISCRETE_DISTRIBUTION<int>(distributionWeights.begin(), distributionWeights.end());

    }

    ComplexController::~ComplexController() {
        if(rewardHistoryStream && storeReward)
            rewardHistoryStream->close();
    }

    void ComplexController::setSensingControllers(std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > sensingControllers) {
        this->sensingControllers = sensingControllers;
    }

    void ComplexController::setPreparatoryControllers(std::vector<KUKADU_SHARED_PTR<kukadu::Controller> > preparatoryControllers) {
        this->preparationControllers = preparatoryControllers;
    }

    void ComplexController::initialize() {

        // create / load sensing database
        createSensingDatabase();

        bool existsPs = fileExists(storePath + "ps");
        string envModelPath = storePath + "envmodels";
        preparePathString(envModelPath);

        if(existsPs) {

            // skill was already initialized and can be loaded again
            if(!isShutUp)
                cout << "(ComplexController) loading existing PS" << endl;

            auto loadLambda = [this] (const std::string& line, const int& level, const int& perceptDimensionality, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator) -> KUKADU_SHARED_PTR<Clip> {

                KukaduTokenizer tok(line, ";");
                string idVec = tok.next();
                string label = tok.next();
                int immunity = atoi(tok.next().c_str());
                if(level == 0) {

                    auto pc = KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(atoi(tok.next().c_str()), label, generator, idVec, immunity));
                    return pc;

                } else if(level == CLIP_H_LEVEL_FINAL) {

                    auto ac = KUKADU_SHARED_PTR<ActionClip>(new ControllerActionClip(atoi(tok.next().c_str()), (this->availablePreparatoryControllers)[label], generator));
                    return ac;

                } else {

                    if(level == 1)
                        return KUKADU_SHARED_PTR<Clip>(new IntermediateEventClip((this->availableSensingControllers)[label],
                                                                                        level, generator, idVec, immunity));

                    return KUKADU_SHARED_PTR<Clip>(new Clip(level, generator, idVec, immunity));

                }

            };

            projSim = KUKADU_SHARED_PTR<ProjectiveSimulator>(new ProjectiveSimulator(shared_from_this(), generator, storePath + "ps", loadLambda));
            // ugly syntax - i have to kill these shared pointers some day
            prepActions = (*((*(projSim->getClipLayers()->end() - 1))->begin()))->getSubClips();
            prepActionsCasted = projSim->getActionClips();
            root = *(projSim->getPerceptClips()->begin());

        } else {

            int currentId = 0;
            KUKADU_SHARED_PTR<vector<int> > clipDimVal = KUKADU_SHARED_PTR<vector<int> >(new vector<int>());
            clipDimVal->push_back(currentId);
            root = KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(0, "root", generator, clipDimVal, INT_MAX));

            vector<double> prepWeights;
            prepActions = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<Clip> > >(new vector<KUKADU_SHARED_PTR<Clip> >());
            prepActionsCasted = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<ActionClip> > >(new vector<KUKADU_SHARED_PTR<ActionClip> >());
            for(int i = 0; i < preparationControllers.size(); ++i) {

                auto prepActionClip = KUKADU_SHARED_PTR<ActionClip>(new ControllerActionClip(i, preparationControllers.at(i), generator));
                prepActions->push_back(prepActionClip);
                prepActionsCasted->push_back(prepActionClip);
                prepWeights.push_back(stdPrepWeight);

            }
            ++currentId;

            for(int i = 0; i < sensingControllers.size(); ++i) {

                KUKADU_SHARED_PTR<SensingController> sensCont = sensingControllers.at(i);

                clipDimVal = KUKADU_SHARED_PTR<vector<int> >(new vector<int>());
                clipDimVal->push_back(currentId);

                KUKADU_SHARED_PTR<Clip> nextSensClip = KUKADU_SHARED_PTR<Clip>(new IntermediateEventClip(sensCont, 1, generator, clipDimVal, INT_MAX));
                ++currentId;
                for(int j = 0; j < sensCont->getSensingCatCount(); ++j, ++currentId) {

                    clipDimVal = KUKADU_SHARED_PTR<vector<int> >(new vector<int>());
                    clipDimVal->push_back(currentId);
                    KUKADU_SHARED_PTR<Clip> nextSubSensClip = KUKADU_SHARED_PTR<Clip>(new Clip(2, generator, clipDimVal, INT_MAX));
                    nextSubSensClip->setChildren(prepActions, prepWeights);
                    nextSensClip->addSubClip(nextSubSensClip, stdPrepWeight);

                }

                double nextWeight = std::exp(senseStretch * max(0.0, sensingWeights.at(i) - 1.0 / sensCont->getSensingCatCount()));
                sensCont->setSimulationClassificationPrecision(min(sensingWeights.at(i) * 100.0, 100.0));
                if(!isShutUp)
                    cout << "(ComplexController) relative weight of sensing action \"" << sensCont->getCaption() << "\" is " << nextWeight << endl;
                root->addSubClip(nextSensClip, nextWeight);

            }

            KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<PerceptClip> > > rootVec = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<PerceptClip> > >(new vector<KUKADU_SHARED_PTR<PerceptClip> >());
            rootVec->push_back(root);

            // skill is used the first time; do initialization
            projSim = KUKADU_SHARED_PTR<ProjectiveSimulator>(new ProjectiveSimulator(shared_from_this(), generator, rootVec, gamma, ProjectiveSimulator::PS_USE_ORIGINAL, false));

        }

        if(fileExists(envModelPath)) {

            // load environment model
            for(auto sens : sensingControllers) {
                auto envModel = KUKADU_SHARED_PTR<ProjectiveSimulator>(new ProjectiveSimulator(nullptr, generator, envModelPath + sens->getCaption()));
                environmentModels.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> >(sens->getCaption(), envModel));
            }

        } else {

            // create environment model
            createDirectory(envModelPath);

            // load environment model
            for(auto sens : sensingControllers) {
                auto envModel = createEnvironmentModelForSensingAction(sens, projSim);
                environmentModels.insert(std::pair<std::string, KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> >(sens->getCaption(), envModel));
                envModel->storePS(envModelPath + sens->getCaption());
            }

        }

        projSim->setBoredom(boredom);

        if(storeReward) {
            rewardHistoryStream = KUKADU_SHARED_PTR<std::ofstream>(new std::ofstream());
            int overWrite = 0;
            if(fileExists(rewardHistoryPath + "history")) {
                cout << "(ComplexController) should reward history file be overwritten? (0 = no / 1 = yes)" << endl;
                cin >> overWrite;
                if(overWrite != 1) {
                    string err = "(ComplexController) reward file already exists. you chose not to overwrite. stopping";
                    cerr << err << endl;
                    throw err;
                }
            }
            rewardHistoryStream->open(rewardHistoryPath.c_str(), ios::trunc);
        }

    }

    void ComplexController::setBoredom(double boredom) {
        projSim->setBoredom(boredom);
    }

    double ComplexController::getPunishReward() {
        return punishReward;
    }

    double ComplexController::getStdReward() {
        return stdReward;
    }

    void ComplexController::setSimulationModeInChain(bool simulationMode) {
        for(int i = 0; i < sensingControllers.size(); ++i) {
            KUKADU_SHARED_PTR<SensingController> sensCont = sensingControllers.at(i);
            sensCont->setSimulationMode(simulationMode);
        }

    }

    KUKADU_SHARED_PTR<kukadu::ProjectiveSimulator> ComplexController::createEnvironmentModelForSensingAction(KUKADU_SHARED_PTR<kukadu::SensingController> sensingAction,
                                                                                          KUKADU_SHARED_PTR<ProjectiveSimulator> projSim) {

        KUKADU_SHARED_PTR<IntermediateEventClip> sensingClip = nullptr;
        auto sensingLayer = projSim->getClipLayers()->at(1);
        for(auto sensingAct : *sensingLayer) {
            if(KUKADU_DYNAMIC_POINTER_CAST<IntermediateEventClip>(sensingAct)->toString() == sensingAction->getCaption()) {
                sensingClip = KUKADU_DYNAMIC_POINTER_CAST<IntermediateEventClip>(sensingAct);
                break;
            }
        }

        if(!sensingClip)
            throw KukaduException("(createEnvironmentModel) sensing action not available");

        auto prepClips = sensingClip->getSubClipByIdx(0)->getSubClips();

        int sensingCatCount = sensingClip->getSubClipCount();
        int prepActionsCount = prepClips->size();

        auto stateClips = sensingClip->getSubClips();

        auto environmentPercepts = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<PerceptClip> > >(new vector<KUKADU_SHARED_PTR<PerceptClip> >());
        auto resultingStatePercepts = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<Clip> > >(new vector<KUKADU_SHARED_PTR<Clip> >());

        auto idVec = KUKADU_SHARED_PTR< vector<int> >(new vector<int>{0, 0});
        for(auto stateClip : *stateClips) {
            auto stateId = stateClip->getClipDimensions()->at(0);
            stringstream s;
            s << "E" << stateId;
            resultingStatePercepts->push_back(KUKADU_SHARED_PTR<ActionClip>(new ActionClip(stateId, idVec->size(), s.str(), generator)));
        }

        for(int stateIdx = 0, overallId = sensingCatCount; stateIdx < sensingCatCount; ++stateIdx) {

            int stateId = stateClips->at(stateIdx)->getClipDimensions()->at(0);
            idVec->at(0) = stateId;

            for(int actId = 0; actId < prepActionsCount; ++actId, ++overallId) {

                idVec->at(1) = prepClips->at(actId)->getClipDimensions()->at(0);

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

    KUKADU_SHARED_PTR<ProjectiveSimulator> ComplexController::getProjectiveSimulator() {
        return projSim;
    }

    void ComplexController::load(std::string path, std::map<std::string, KUKADU_SHARED_PTR<kukadu::SensingController> > availableSensingControllers, std::map<std::string, KUKADU_SHARED_PTR<kukadu::Controller> > availablePreparatoryControllers) {

        sensingControllers.clear();
        if(prepActions)
            prepActions->clear();
        if(prepActionsCasted)
            prepActionsCasted->clear();
        preparationControllers.clear();

        preparePathString(path);
        string compositionDestination = path + "composition";

        ifstream compositionFile;
        compositionFile.open(compositionDestination);

        int controllerMode = -1;
        string line = "";
        while(getline(compositionFile, line)) {

            if(!line.compare(FILE_SENSING_PREFIX))
                controllerMode = 1;
            else if(!line.compare(FILE_PREP_PREFIX))
                controllerMode = 2;
            else if(!line.compare(FILE_END_PREFIX))
                controllerMode = 10;
            else if(line.compare("")) {
                // we are reading sensing controllers
                if(controllerMode == 1) {
                    sensingControllers.push_back(availableSensingControllers[line]);
                } else if(controllerMode == 2) {
                    preparationControllers.push_back(availablePreparatoryControllers[line]);
                } else if(controllerMode == 10) {
                    throw KukaduException("(ComplexController) malformed composition file");
                }
            }

        }

        storePath = path;
        this->availableSensingControllers = availableSensingControllers;
        this->availablePreparatoryControllers = availablePreparatoryControllers;
        initialize();

    }

    void ComplexController::store() {
        store(storePath);
    }

    void ComplexController::store(std::string destination) {

        preparePathString(destination);
        string psDestination = destination + "ps";
        string compositionDestination = destination + "composition";
        projSim->storePS(psDestination);
        ofstream compositionFile;
        compositionFile.open(compositionDestination);
        compositionFile << FILE_SENSING_PREFIX << endl;
        for(auto sensCont : sensingControllers)
            compositionFile << sensCont->getCaption() << endl;

        compositionFile << FILE_PREP_PREFIX << endl;
        for(auto prepCont : preparationControllers)
            compositionFile << prepCont->getCaption() << endl;

        compositionFile.close();

    }

    void ComplexController::storeNextIteration() {
        stringstream s;
        s << storePath + "ps" << currentIterationNum;
        projSim->storePS(s.str());
    }

    int ComplexController::getDimensionality() {
        if(!isShutUp)
            cout << "(ComplexController) get dimensionality got called" << endl;
        return 1;
    }

    // same percept must always have same id
    KUKADU_SHARED_PTR<PerceptClip> ComplexController::generateNextPerceptClip(int immunity) {
        return root;
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > ComplexController::generateActionClips() {
        return prepActionsCasted;
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > ComplexController::generatePerceptClips() {
        vector<KUKADU_SHARED_PTR<PerceptClip> >* rootRet;
        rootRet->push_back(root);
        return KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > >(rootRet);
    }

    double ComplexController::computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction) {

        int worked = 0;
        int executeIt = 0;

        double retReward = 0.0;

        KUKADU_SHARED_PTR<vector<int> > intermed = projSim->getIntermediateHopIdx();
        KUKADU_SHARED_PTR<IntermediateEventClip> sensClip = KUKADU_DYNAMIC_POINTER_CAST<IntermediateEventClip>(providedPercept->getSubClipByIdx(intermed->at(1)));
        KUKADU_SHARED_PTR<SensingController> sensCont = sensClip->getSensingController();

        KUKADU_SHARED_PTR<ControllerActionClip> castedAction = KUKADU_DYNAMIC_POINTER_CAST<ControllerActionClip>(takenAction);

        int predictedClassIdx = intermed->at(2);
        int takenActionIdx = intermed->at(3);

        if(!getSimulationMode()) {

            cout << "selected sensing action \"" << *sensClip << "\" resulted in predicted class " << intermed->at(2) << " and preparation action \"" << *takenAction << "\"" << endl;

            castedAction->performAction();

            cout << "(ComplexController) do you want to execute complex action now? (0 = no / 1 = yes)" << endl;
            cin >> executeIt;

            if(executeIt)
                executeComplexAction();

            cout << "did the complex action succeed? (0 = no / 1 = yes)" << endl;
            cin >> worked;

            if(worked) {
                cout << "preparation action worked; rewarded with " << stdReward << endl;
                retReward = stdReward;
            } else {
                cout << "preparation action didn't work; no reward given" << endl;
                retReward = punishReward;
            }

        } else {

            retReward = getSimulatedRewardInternal(sensCont, providedPercept, castedAction->getActionController(), predictedClassIdx, takenActionIdx);

        }

        if(storeReward)
            *rewardHistoryStream << retReward << "\t" << *sensClip << "\t" << predictedClassIdx << "\t\t" << *takenAction << endl;

        return retReward;

    }

    double ComplexController::getSimulatedRewardInternal(KUKADU_SHARED_PTR<SensingController> usedSensingController, KUKADU_SHARED_PTR<kukadu::PerceptClip> providedPercept, KUKADU_SHARED_PTR<kukadu::Controller> takenAction, int sensingClassIdx, int prepContIdx) {

        // simulates non-perfect complex controller
        int failed = simSuccDist(*generator);
        if(failed)
            return getPunishReward();

        double simReward = getSimulatedReward(usedSensingController, providedPercept, takenAction, sensingClassIdx, prepContIdx);

        return simReward;

    }

    int ComplexController::getNextSimulatedGroundTruth(KUKADU_SHARED_PTR<SensingController> sensCont) {
        return sensCont->createRandomGroundTruthIdx();
    }

    KUKADU_SHARED_PTR<kukadu_mersenne_twister> ComplexController::getGenerator() {
        return gen;
    }

    KUKADU_SHARED_PTR<ControllerResult> ComplexController::performAction() {

        // if simulation - set observed state as ground truth for each sensing action (it is not yet know, which sensing action will be selected)
        if(getSimulationMode()) {
            for(auto sensCont : sensingControllers) {
                auto nextGroundTruth = getNextSimulatedGroundTruth(sensCont);
                sensCont->setSimulationGroundTruth(nextGroundTruth);
            }
        }

        ++currentIterationNum;
        projSim->performRandomWalk();
        tuple<bool, double, vector<int> > actionRes = projSim->performRewarding();
        auto wasBored = std::get<0>(actionRes);
        auto reward = std::get<1>(actionRes);
        auto hopPath = std::get<2>(actionRes);

        if(wasBored) {

            auto firstPercept = projSim->getPerceptClips()->at(hopPath.at(hopPath.at(0)));
            auto currentClip = KUKADU_DYNAMIC_POINTER_CAST<Clip>(firstPercept);
            vector<KUKADU_SHARED_PTR<Clip> > clipPath = {currentClip};
            for(int i = 1; i < hopPath.size(); ++i) {
                int nextHop = hopPath.at(i);
                currentClip = currentClip->getSubClipByIdx(nextHop);
                clipPath.push_back(currentClip);
            }

            auto stateClip = *(clipPath.end() - 2);
            auto sensingClip = *(clipPath.begin() + 1);
            auto possiblePaths = computeEnvironmentPaths(sensingClip, stateClip, maxEnvPathLength);

            std::sort(possiblePaths.begin(), possiblePaths.end(), [] (std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> > > p1, std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> > > p2) {
                          return std::get<0>(p1) > std::get<0>(p2);
                      });

            computeTotalPathCost(possiblePaths);

            std::sort(possiblePaths.begin(), possiblePaths.end(), [] (std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> > > p1, std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> > > p2) {
                          return std::get<0>(p1) > std::get<0>(p2);
                      });

            auto selectedPath = possiblePaths.at(0);
            for(auto cl : possiblePaths) {
                auto targetPercept = get<1>(cl);
                cout << *targetPercept << " " << *stateClip << endl;
                if(*targetPercept != *stateClip) {
                    selectedPath = cl;
                    break;
                }
            }

            if(!isShutUp) {
                cout << "(ComplexController) got bored" << endl;
                cout << "selected path info:" << endl;
                cout << "target clip: " << *get<1>(selectedPath) << endl;
                cout << "selected path: ";
                for(auto cl : get<2>(selectedPath))
                    cout << *cl << " - ";
                cout << endl;
            }

            // if controller is in real execution mode, execute the preparatory path
            if(!getSimulationMode()) {

                // executing path
                auto path = get<2>(selectedPath);
                for(int i = 0; i < path.size(); ++i) {
                    auto cl = path.at(i);
                    // if clip number is odd, that clip is a preparatory action
                    if(i % 2) {
                        if(!isShutUp)
                            cout << "(ComplexController) next action is " << *cl;
                        KUKADU_DYNAMIC_POINTER_CAST<ControllerActionClip>(cl)->performAction();
                    } else {
                        if(!isShutUp)
                            cout << "(ComplexController) current state should be " << *cl;
                    }
                }

            } else {
            // if controller is in simulation mode, check ground truth

                auto groundTruthStateClip = stateClip;
                cout << *stateClip << " ";
                auto path = get<2>(selectedPath);
                for(int i = 0; i < path.size(); ++i) {
                    auto cl = path.at(i);
                    // if clip number is odd, that clip is a preparatory action
                    if(i % 2) {
                        if(!isShutUp)
                            cout << "(ComplexController) next action is " << *cl;
                        groundTruthStateClip = computeGroundTruthTransition(sensingClip, groundTruthStateClip, cl);
                        cout << *cl << " " << *groundTruthStateClip << endl;
                    } else {
                        if(!isShutUp)
                            cout << "(ComplexController) current state should be " << *cl;
                    }
                }

            }


            // todo (1):
            // - sense state after manipulation actions
            // - replace the new state in the performed path (is it enough to replace intermediate hop idx?)
            // - switch off boredom
            // - perform rewarding
            //      a) execution: execute complex action and measure reward
            //      b) simulation: sample from complex action success rate in ground truth state

            // todo (2):
            // it seems like the boredom is not working in real mode (random walk needs to stopped in the middle when it is bored)
            // boredom is only computed in performRewarding() --> it has to be done in performRandomWalk()

        }

        KUKADU_SHARED_PTR<ControllerResult> ret = KUKADU_SHARED_PTR<ControllerResult>(new ControllerResult(vec(), vector<vec>(),
                                                                                                           (reward > 0) ? true : false, wasBored,
                                                                                                           *(projSim->getIntermediateHopIdx())));
        return ret;

    }

    void ComplexController::printPaths(std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> > > >& paths) {
        for(auto path : paths) {
            cout << get<0>(path) << "; " << *get<1>(path) << ": ";
            for(auto cl : get<2>(path))
                cout << *cl << " - ";
            cout << endl;
        }
    }

    void ComplexController::computeTotalPathCost(std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> > > >& paths) {

        for(auto& path : paths) {

            double& pathCost = std::get<0>(path);
            KUKADU_SHARED_PTR<Clip> finalStateClip = std::get<1>(path);
            auto& clipPath = std::get<2>(path);
            double finalClipEntropy = finalStateClip->computeSubEntropy();

            // there are state clips and action clips mixed - the first and the last clips are state clips, so the
            // size has to be reduced by one --> then the number of taken actions is half of that size
            int pathLength = (clipPath.size() - 1) / 2 + 1;

            // update the cost of the path
            pathCost = finalClipEntropy * pathCost + pathLengthCost / pathLength;

        }

    }

    std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> > > > ComplexController::computeEnvironmentPaths(
            KUKADU_SHARED_PTR<Clip> sensingClip, KUKADU_SHARED_PTR<Clip> stateClip, int maxPathLength) {

        auto sensingId = sensingClip->toString();
        auto stateId = stateClip->getClipDimensions()->at(0);

        std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> > > > allPaths;
        std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> > > > lastIterationPaths;
        std::vector<std::tuple<double, KUKADU_SHARED_PTR<Clip>, std::vector<KUKADU_SHARED_PTR<Clip> > > > lastIterationPathsOld;

        // initialize with paths of length 0
        std::vector<KUKADU_SHARED_PTR<Clip> > path = {stateClip};
        allPaths.push_back(std::make_tuple(1.0, stateClip, path));
        lastIterationPaths.push_back(std::make_tuple(1.0, stateClip, path));

        for(int i = 0; i < maxPathLength; ++i) {

            lastIterationPathsOld = lastIterationPaths;
            lastIterationPaths.clear();
            // check every path and see how it can be made longer
            for(auto path : lastIterationPathsOld) {

                double currentConfidence = std::get<0>(path);
                auto currentState = std::get<1>(path);
                stateId = currentState->getClipDimensions()->at(0);

                // for every possible transition, analyse how confidently another state can be reached
                auto stateClips =  environmentModels[sensingId]->retrieveClipsOnLayer({stateId, ProjectiveSimulator::IGNORE_ID}, 0);
                for(auto state : stateClips) {

                    // copy old path again
                    auto currentPath = std::get<2>(path);
                    auto stateTransition = computeEnvironmentTransitionConfidence(state);
                    double transitionConfidence = stateTransition.first;
                    auto resultingStateId = stateTransition.second;
                    auto resultingStateClip = projSim->retrieveClipsOnLayer({resultingStateId}, 2).at(0);
                    int actionId = state->getClipDimensions()->at(1);
                    auto usedActionClip = projSim->retrieveClipsOnLayer({actionId}, 3).at(0);
                    currentPath.push_back(usedActionClip);
                    currentPath.push_back(resultingStateClip);

                    double nextConfidence = currentConfidence * transitionConfidence;

                    if(nextConfidence > 0) {
                        allPaths.push_back(std::make_tuple(nextConfidence, resultingStateClip, currentPath));
                        lastIterationPaths.push_back(std::make_tuple(nextConfidence, resultingStateClip, currentPath));
                    }

                }

            }

        }

        return allPaths;

    }

    pair<double, int> ComplexController::computeEnvironmentTransitionConfidence(KUKADU_SHARED_PTR<Clip> stateClip) {

        double stateEntropy = stateClip->computeSubEntropy();
        double confidenceProb = 1.0 - stateEntropy / log2(stateClip->getSubClipCount());
        auto likeliestResult = stateClip->getLikeliestChild();
        int envId = atoi(likeliestResult->toString().substr(1).c_str());
        return std::make_pair(confidenceProb, envId);

    }

    void ComplexController::setTrainingMode(bool doTraining) {
        projSim->setTrainingMode(doTraining);
    }

    void ComplexController::createSensingDatabase() {

        createSensingDatabase(sensingControllers);

    }

    void ComplexController::createSensingDatabase(std::vector<KUKADU_SHARED_PTR<SensingController> > sensingControllers) {

        sensingWeights.clear();
        for(int i = 0; i < sensingControllers.size(); ++i) {
            double validationScore = sensingControllers.at(i)->createDataBase();
            sensingWeights.push_back(validationScore);
        }

    }

}
