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
                                         int simulationFailingProbability)
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

        bool existsPs = fileExists(storePath + "ps");

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

            // create ecm
            createSensingDatabase();

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

            if(executeIt) {
                executeComplexAction();
            }

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

            sensCont->setSimulationGroundTruth(getNextSimulatedGroundTruth(sensCont));
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

    KUKADU_SHARED_PTR<ControllerResult> ComplexController::performAction() {

        ++currentIterationNum;
        projSim->performRandomWalk();
        pair<bool, double> actionRes = projSim->performRewarding();
        bool wasBored = actionRes.first;
        double reward = actionRes.second;

        if(wasBored && !isShutUp)
            cout << "(ComplexController) got bored" << endl;

        KUKADU_SHARED_PTR<ControllerResult> ret = KUKADU_SHARED_PTR<ControllerResult>(new ControllerResult(vec(), vector<vec>(), (reward > 0) ? true : false, wasBored));
        return ret;

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
