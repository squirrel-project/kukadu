#include "projectivesimulator.h"

#include <utility>
#include <iostream>
#include <fstream>

#include "../utils/Tokenizer.h"

#include <fstream>
#include <iostream>

using namespace std;

ProjectiveSimulator::ProjectiveSimulator(std::shared_ptr<Reward> reward, std::shared_ptr<std::mt19937> generator, std::string file) {

    this->reward = reward;
    this->generator = generator;

    this->lastActionClip = nullptr;
    this->lastPerceptClip = nullptr;

    actionClips = std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>>(new std::vector<std::shared_ptr<ActionClip>>());
    perceptClips = std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>>(new std::vector<std::shared_ptr<PerceptClip>>());
    clipLayers = std::shared_ptr<std::vector<std::shared_ptr<std::set<std::shared_ptr<Clip>, clip_compare>>>>(new std::vector<std::shared_ptr<std::set<std::shared_ptr<Clip>, clip_compare>>>());

    intermediateHops = std::shared_ptr<vector<int>>(new vector<int>());

    string line = "";
    ifstream inputFile;
    inputFile.open(file);

    // check version
    getline(inputFile, line);
    if(!line.compare("V1.0")) {

        // ignore "general properties" line
        getline(inputFile, line);

        // operation mode
        getline(inputFile, line);
        Tokenizer tok(line, "=");
        tok.next(); operationMode = atoi(tok.next().c_str());

        // use ranking?
        getline(inputFile, line);
        tok = Tokenizer(line, "=");
        tok.next(); useRanking = (atoi(tok.next().c_str()))?true:false;

        // gamma
        getline(inputFile, line);
        tok = Tokenizer(line, "=");
        tok.next(); gamma = atof(tok.next().c_str());

        // max number of clips (ignored without ranking)
        getline(inputFile, line);
        tok = Tokenizer(line, "=");
        tok.next(); maxNumberOfClips = atoi(tok.next().c_str());

        // immunity threshhold (ignored without ranking)
        getline(inputFile, line);
        tok = Tokenizer(line, "=");
        tok.next(); immunityThresh = atoi(tok.next().c_str());

        // levels
        getline(inputFile, line);
        tok = Tokenizer(line, "=");
        tok.next(); levels = atoi(tok.next().c_str());

        for(int i = 0; i < levels; ++i)
            clipLayers->push_back(std::shared_ptr<set<std::shared_ptr<Clip>, clip_compare>>(new set<std::shared_ptr<Clip>, clip_compare>()));

        getline(inputFile, line);
        getline(inputFile, line);

        // load the clips
        int currentLayer = 0;
        bool isFirstPercept = true;
        int perceptDimensionality = 0;
        while(getline(inputFile, line) && line.compare("")) {

            // check if its a layer line
            tok = Tokenizer(line, "=");
            string nextToken = tok.next();
            if(!nextToken.compare("layer")) {

                string layerString = tok.next();
                currentLayer = atoi(layerString.c_str());

            } else {

                // it is no t a layer line (must be a clip line
                tok = Tokenizer(line, ";");
                shared_ptr<Clip> nextClip = nullptr;

                // first line is the id vector
                if(currentLayer == 0) {

                    string idVec = tok.next();
                    string label = tok.next();
                    int immunity = atoi(tok.next().c_str());
                    std::shared_ptr<PerceptClip> pc = std::shared_ptr<PerceptClip>(new PerceptClip(atoi(tok.next().c_str()), label, generator, idVec, immunity));
                    nextClip = pc;

                    if(isFirstPercept) {

                        isFirstPercept = false;
                        perceptDimensionality = pc->getDimensionality();

                    }

                    perceptClips->push_back(pc);

                } else if(currentLayer == CLIP_H_LEVEL_FINAL) {

                    // id vec is not useful
                    tok.next();
                    string label = tok.next();
                    int immunity = atoi(tok.next().c_str());
                    std::shared_ptr<ActionClip> ac = std::shared_ptr<ActionClip>(new ActionClip(atoi(tok.next().c_str()), perceptDimensionality, label, generator));
                    nextClip = ac;
                    actionClips->push_back(ac);

                } else {

                    string idVec = tok.next();
                    int immunity = atoi(tok.next().c_str());
                    std::shared_ptr<Clip> c = nextClip = std::shared_ptr<Clip>(new Clip(currentLayer, generator, idVec, immunity));

                }

                int clipLevel = currentLayer;
                if(clipLevel != CLIP_H_LEVEL_FINAL)
                    clipLayers->at(clipLevel)->insert(nextClip);
                else
                    clipLayers->at(clipLayers->size() - 1)->insert(nextClip);

            }

        }

        getline(inputFile, line);

        // load the clips
        currentLayer = 0;
        shared_ptr<Clip> currentParent = nullptr;
        std::vector<double> newChildrenWeights;
        std::shared_ptr<std::vector<std::shared_ptr<Clip>>> newChildren = std::shared_ptr<std::vector<std::shared_ptr<Clip>>>(new std::vector<std::shared_ptr<Clip>>());
        while(getline(inputFile, line)) {

            // check if its a layer line
            tok = Tokenizer(line, "=");
            string nextToken = tok.next();
            if(!nextToken.compare("layer")) {

                string layerString = tok.next();
                currentLayer = atoi(layerString.c_str());

            } else {

                // a new parent will be provided next
                if(!line.compare("")) {

                    currentParent->setChildren(newChildren, newChildrenWeights);
                    currentParent = nullptr;
                    newChildrenWeights.clear();
                    newChildren = std::shared_ptr<std::vector<std::shared_ptr<Clip>>>(new std::vector<std::shared_ptr<Clip>>());

                }
                // its a new parent clip
                else if(line.find(':') != string::npos) {

                    currentParent = findClipByIdVec(Clip::getIdVectorFromString(line.substr(0, line.size() - 1)));

                } else if(line.find(';') != string::npos) {

                    // it must be a new child clip
                    tok = Tokenizer(line, ";");
                    string idVecString = tok.next();
                    double connectionWeight = atof(tok.next().c_str());
                    shared_ptr<Clip> currentChild = findClipByIdVec(Clip::getIdVectorFromString(idVecString));
                    newChildren->push_back(currentChild);
                    newChildrenWeights.push_back(connectionWeight);

                }

            }

        }

    } else {
        throw "PS file version cannot be handled";
    }

}

int ProjectiveSimulator::getIdVecLevel(std::shared_ptr<std::vector<int> > idVec) {

    int retCount = 0;
    for(int val : *idVec)
        if(val == CLIP_H_HASH_VAL)
            retCount++;

    return retCount;

}

std::shared_ptr<Clip> ProjectiveSimulator::findClipInLevelByIdVec(std::shared_ptr<std::vector<int>> idVec, int level) {
    std::shared_ptr<std::set<std::shared_ptr<Clip>, clip_compare>> currentLayer = clipLayers->at(level);
    for(shared_ptr<Clip> c : *currentLayer) {
        shared_ptr<vector<int>> clipDim = c->getClipDimensions();
        if(Clip::compareIdVecs(clipDim, idVec)) {
            return c;
        }
    }
    return nullptr;
}

std::shared_ptr<Clip> ProjectiveSimulator::findClipByIdVec(std::shared_ptr<std::vector<int>> idVec) {

    if(operationMode == PS_USE_ORIGINAL) {

        for(int level = 0; level < clipLayers->size() - 1; ++level) {
            std::shared_ptr<Clip> retVal = findClipInLevelByIdVec(idVec, level);
            if(retVal != nullptr)
                return retVal;
        }

    } else if(operationMode == PS_USE_GEN) {
        // this works only for generalization where the number of wildcards says something about the level
        int level = getIdVecLevel(idVec);
        return findClipInLevelByIdVec(idVec, level);
    }

    for(shared_ptr<Clip> ac : *actionClips) {
        shared_ptr<vector<int>> clipDim = ac->getClipDimensions();
        if(Clip::compareIdVecs(clipDim, idVec)) {
            return ac;
        }
    }

    return nullptr;

}

void ProjectiveSimulator::setBoredom(double boredom) {
    if(boredom <= 0.0) {
        this->boredom = 0.0;
        this->useBoredom = false;
    } else {
        this->boredom = boredom;
        this->useBoredom = true;
    }
}

void ProjectiveSimulator::setTrainingMode(bool doTraining) {
    this->doTraining = doTraining;
}

void ProjectiveSimulator::construct(std::shared_ptr<Reward> reward, std::shared_ptr<std::mt19937> generator, double gamma, int operationMode, bool useRanking) {

    this->boredom = 0.0;
    this->doTraining = true;
    this->useBoredom = false;

    this->useRanking = useRanking;
    this->operationMode = operationMode;
    this->gamma = gamma;
    intermediateHops = std::shared_ptr<vector<int>>(new vector<int>());

    this->lastActionClip = nullptr;
    this->lastPerceptClip = nullptr;

    this->reward = reward;
    this->maxNumberOfClips = PS_MAX_NUMBER_OF_CLIPS;

    this->generator = generator;
    intDist = std::uniform_int_distribution<int>(0, perceptClips->size() - 1);

    clipLayers = std::shared_ptr<std::vector<std::shared_ptr<std::set<std::shared_ptr<Clip>, clip_compare>>>>(new std::vector<std::shared_ptr<std::set<std::shared_ptr<Clip>, clip_compare>>>());

    for(int i = 0; i < levels + 1; ++i)
        clipLayers->push_back(std::shared_ptr<set<std::shared_ptr<Clip>, clip_compare>>(new set<std::shared_ptr<Clip>, clip_compare>()));

    clipLayers->at(0)->insert(perceptClips->begin(), perceptClips->end());
    clipLayers->at(clipLayers->size() - 1)->insert(actionClips->begin(), actionClips->end());

    lastGeneralizedPercept = nullptr;

}

ProjectiveSimulator::ProjectiveSimulator(std::shared_ptr<Reward> reward, std::shared_ptr<std::mt19937> generator, double gamma, int operationMode, bool useRanking) {

    this->perceptClips = reward->generatePerceptClips();
    this->actionClips = reward->generateActionClips();

    levels = 0;
    if(operationMode == PS_USE_ORIGINAL)
        levels = 1;
    else if(operationMode == PS_USE_GEN)
        // + 1 for the (#, #, #, ...) layer
        levels = reward->getDimensionality() + 1;

    std::shared_ptr<std::vector<std::shared_ptr<Clip>>> clipActionClips = shared_ptr<std::vector<std::shared_ptr<Clip>>>(new std::vector<std::shared_ptr<Clip>>());
    for(std::shared_ptr<Clip> t : *actionClips)
        clipActionClips->push_back(dynamic_pointer_cast<Clip>(t));

    // has to be changed for multi level ps
    for(std::shared_ptr<Clip> currentClip : *perceptClips)
        currentClip->setChildren(clipActionClips);

    construct(reward, generator, gamma, operationMode, useRanking);

}

ProjectiveSimulator::ProjectiveSimulator(std::shared_ptr<Reward> reward, std::shared_ptr<std::mt19937> generator,
                    std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>> network,
                    double gamma, int operationMode, bool useRanking) {

    this->perceptClips = network;

    // set levels and action clips
    // walk down to last percept
    levels = 0;
    shared_ptr<Clip> lastClip = nullptr;
    shared_ptr<Clip> currClip = perceptClips->at(0);
    while(currClip->getSubClipCount()) {
        lastClip = currClip;
        currClip = currClip->getSubClipByIdx(0);
        ++levels;
    }

    actionClips = shared_ptr<vector<shared_ptr<ActionClip>>>(new vector<shared_ptr<ActionClip>>());
    for(int i = 0; i < lastClip->getSubClipCount(); ++i) {
        shared_ptr<ActionClip> nextActClip = dynamic_pointer_cast<ActionClip>(lastClip->getSubClipByIdx(i));
        actionClips->push_back(nextActClip);
    }

    construct(reward, generator, gamma, operationMode, useRanking);

    // fill inbetween layers (clipLayers)
    for(shared_ptr<PerceptClip> pc : *perceptClips)
        fillClipLayersFromNetwork(pc);

}

void ProjectiveSimulator::fillClipLayersFromNetwork(std::shared_ptr<Clip> cl) {

    int level = cl->getLevel();

    if(level != CLIP_H_LEVEL_FINAL) {
        clipLayers->at(level)->insert(cl);
        for(int i = 0; i < cl->getSubClipCount(); ++i)
            fillClipLayersFromNetwork(cl->getSubClipByIdx(i));
    } else {
        clipLayers->at(clipLayers->size() - 1)->insert(cl);
    }


}

std::shared_ptr<std::vector<int>> ProjectiveSimulator::getIntermediateHopIdx() {
    return intermediateHops;
}

ProjectiveSimulator::~ProjectiveSimulator() {

}

void ProjectiveSimulator::eliminateClip(std::shared_ptr<Clip> currClip) {

    int level = currClip->getLevel();
    std::shared_ptr<set<std::shared_ptr<Clip>, clip_compare>> currLayer = clipLayers->at(level);
    currLayer->erase(currClip);
    std::set<std::shared_ptr<Clip>> parents = std::set<std::shared_ptr<Clip>>(currClip->getParents()->begin(), currClip->getParents()->end());

    for(std::shared_ptr<Clip> parent : parents)
        parent->removeSubClip(currClip);

    if(level == 0 && operationMode != PS_USE_ORIGINAL)
        perceptClips->erase(std::find(perceptClips->begin(), perceptClips->end() + 1, currClip));

    currClip->removeAllSubClips();

}

int ProjectiveSimulator::getStandardImmunity() {
    return immunityThresh;
}

void ProjectiveSimulator::cleanByRank() {

    int clipNumber = rankVec.size();

    int alreadyDeleted = 0;
    int toDelete = clipNumber - maxNumberOfClips;

    if(clipNumber > maxNumberOfClips) {

        for(int i = 0; i < clipNumber; ++i) {

            std::shared_ptr<Clip> currClip = rankVec.at(i).second;
            if(!currClip->isImmune()) {

                ++alreadyDeleted;
                eliminateClip(currClip);
                rankVec.erase(rankVec.begin() + i);
                --i;
                --clipNumber;

                if(alreadyDeleted >= toDelete)
                    return;

            }
        }
    }

    // if too many clips were immune, then start deleting the rest
    if(alreadyDeleted < toDelete) {

        for(int i = 0; alreadyDeleted < toDelete; ++i, ++alreadyDeleted) {

            std::shared_ptr<Clip> currClip = rankVec.at(i).second;
            eliminateClip(currClip);

            rankVec.erase(rankVec.begin() + i);
            --i;

        }

    }

}

void ProjectiveSimulator::setStandardImmunity(int immunity) {
    this->immunityThresh = immunity;
}

void ProjectiveSimulator::setMaxNumberOfClips(int maxNumberOfClips) {
    this->maxNumberOfClips = maxNumberOfClips;
}

std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> ProjectiveSimulator::getActionClips() {
    return actionClips;
}

std::shared_ptr<ActionClip> ProjectiveSimulator::performRandomWalk() {

    lastClipBeforeAction = nullptr;
    intermediateHops->clear();
    std::shared_ptr<Clip> previousClip = nullptr;
    std::shared_ptr<Clip> currentClip = nullptr;

    if(operationMode == PS_USE_GEN) {
        if(!lastGeneralizedPercept) {
            cerr << "(ProjectiveSimulator) you have to generalize before you walk" << endl;
            throw "(ProjectiveSimulator) you have to generalize before you walk";
        } else {
            currentClip = lastGeneralizedPercept;
        }
    } else if(operationMode == PS_USE_ORIGINAL) {
        currentClip = reward->generateNextPerceptClip(immunityThresh);
    }

    std::vector<std::shared_ptr<PerceptClip>>::iterator it = std::find(perceptClips->begin(), perceptClips->end() + 1, currentClip);
    int previousIdx = it - perceptClips->begin();
    lastPerceptClip = std::dynamic_pointer_cast<PerceptClip>(currentClip);

    while(previousClip != currentClip) {
        intermediateHops->push_back(previousIdx);
        pair<int, std::shared_ptr<Clip>> nextHop;
        lastClipBeforeAction = previousClip;
        previousClip = currentClip;
        nextHop = currentClip->jumpNextRandom();
        previousIdx = nextHop.first;
        currentClip = nextHop.second;

    }

    lastActionClip = std::dynamic_pointer_cast<ActionClip>(currentClip);
    return lastActionClip;

}

double ProjectiveSimulator::computeBoredem(std::shared_ptr<Clip> clip) {
    
    double entropy = clip->computeSubEntropy();
    double numberOfSubclips = clip->getSubClipCount();

    // cout << "entropy: " << entropy << "; log2: " << log2(numberOfSubclips) << "; " << "; boredomConst: " << boredom << "; ";

    // b * (1 - H / H_max) = b * (1 - H / log2(N))
    double clipBoredom = boredom * (1.0 - entropy / log2(numberOfSubclips));

    return clipBoredom;

}

pair<bool, double> ProjectiveSimulator::performRewarding() {

    int beingBored = 0;
    if(useBoredom) {

        double boredomScore = computeBoredem(lastClipBeforeAction);
        vector<double> boredomDistWeights = {boredomScore, 1 - boredomScore};
        discrete_distribution<int> boredomDist = discrete_distribution<int>(boredomDistWeights.begin(), boredomDistWeights.end());
        beingBored =  1 - boredomDist(*generator);

        // cout << "boredomScore: " << boredomScore << "; " << "beingBored: " << beeingBored << endl;

    }

    double computedReward = 0.0;
    if(!beingBored) {

        computedReward = reward->computeReward(lastPerceptClip, lastActionClip);

        if(doTraining) {
            for(std::shared_ptr<set<std::shared_ptr<Clip>, clip_compare>> currLevel : *clipLayers) {

                for(std::shared_ptr<Clip> currClip : *currLevel) {

                    currClip->updateWeights(computedReward, gamma);

                    // decrease immunity
                    if(useRanking)
                        currClip->decreaseImmunity();

                }
            }

            if(useRanking) {

                computeRankVec();
                cleanByRank();

            }
        }

    }

    return make_pair(beingBored, computedReward);

}

void ProjectiveSimulator::generalize(std::shared_ptr<PerceptClip> nextClip) {

    lastGeneralizedPercept = nextClip;

    std::shared_ptr<std::set<std::shared_ptr<Clip>, clip_compare>> toConnect = createNewClips(nextClip);

    // first connect everything...
    for(std::shared_ptr<Clip> con : *toConnect) {

        if(PS_PRINT_DEBUG_INFO)
            cout << "calling connect function for " << *con << endl;

        if(useRanking)
            con->setPreviousRank();

        connectNewClip(con);

    }

    //delete toConnect;
    toConnect = nullptr;

}

void ProjectiveSimulator::printWeights() {

    int level = 0;
    for(std::shared_ptr<set<std::shared_ptr<Clip>, clip_compare>> currLevel : *clipLayers) {

        cout << "clips on layer " << level << endl << "=========================" << endl;

        for(std::shared_ptr<Clip> currClip : *currLevel) {

            int subClipCount = currClip->getSubClipCount();
            for(int i = 0; i < subClipCount; ++i) {
                cout << *currClip << " --> " << *currClip->getSubClipByIdx(i) << " with weight " << currClip->getWeightByIdx(i) << endl;
            }

        }

        ++level;

    }

}

void ProjectiveSimulator::connectNewClip(std::shared_ptr<Clip> conClip) {

    int currentLevel = 0;
    for(std::shared_ptr<set<std::shared_ptr<Clip>, clip_compare>> currLevel : *clipLayers) {

        if(currLevel->size() > 0 && conClip->getLevel() != currentLevel) {

            for(std::shared_ptr<Clip> currClip : *currLevel) {

                // if they are compatible
                if(currClip->isCompatibleSubclip(conClip)) {
                    currClip->addSubClip(conClip, CLIP_H_STD_WEIGHT);
                    conClip->addParent(currClip);
                } else if(conClip->isCompatibleSubclip(currClip)) {
                    conClip->addSubClip(currClip, CLIP_H_STD_WEIGHT);
                    currClip->addParent(conClip);
                } else {
                    // dont connect, they are not compatible
                }

            }

        }

        ++currentLevel;

    }

}

std::shared_ptr<std::set<std::shared_ptr<Clip>, clip_compare>> ProjectiveSimulator::createNewClips(std::shared_ptr<PerceptClip> newClip) {

    std::shared_ptr<set<std::shared_ptr<Clip>, clip_compare>> conClips = std::shared_ptr<set<std::shared_ptr<Clip>, clip_compare>>(new set<std::shared_ptr<Clip>, clip_compare>());

    // insert new percept clip
    pair<set<std::shared_ptr<Clip>, clip_compare>::iterator, bool> inserted = clipLayers->at(0)->insert(newClip);

    // check if new clip was already there
    if(inserted.second) {

        // if it wasnt there yet, add it to the new clips that have to be freshly connected
        conClips->insert(newClip);
        perceptClips->push_back(std::dynamic_pointer_cast<PerceptClip>(newClip));

        // check in each level, if there will be new clips
        for(std::shared_ptr<set<std::shared_ptr<Clip>, clip_compare>> currLevel : *clipLayers) {

            std::shared_ptr<Clip> firstClipOnLevel = nullptr;
            if(currLevel->size())
                firstClipOnLevel = *(currLevel->begin());
            if(currLevel->size() && firstClipOnLevel->getLevel() != CLIP_H_LEVEL_FINAL) {

                for(std::shared_ptr<Clip> currClip : *currLevel) {

                    // create new clip that gets generated as a cascade
                    std::shared_ptr<Clip> nextClip = currClip->compareClip(newClip);

                    // check if already there and insert it if not
                    pair<set<std::shared_ptr<Clip>, clip_compare>::iterator, bool> inserted2 = clipLayers->at(nextClip->getLevel())->insert(nextClip);

                    // if new && not already there
                    if(inserted2.second && nextClip != currClip) {

                        nextClip = *inserted2.first;

                        // insert to the new set of clips that should be connected
                        pair<set<std::shared_ptr<Clip>, clip_compare>::iterator, bool> inserted3 = conClips->insert(nextClip);


                    }
                    // if same clip is there already, but new instance, then delete the instance
                    else if(newClip != nextClip) {
                    //    delete nextClip;
                        nextClip = nullptr;
                    }

                }

            }
            else break;
        }

    }

    return conClips;

}

std::shared_ptr<std::vector<std::shared_ptr<std::set<std::shared_ptr<Clip>, clip_compare>>>> ProjectiveSimulator::getClipLayers() {
    return clipLayers;
}

std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>> ProjectiveSimulator::getPerceptClips() {
    return perceptClips;
}

bool compareRanks(std::pair<double, std::shared_ptr<Clip>> p1, std::pair<double, std::shared_ptr<Clip>> p2) {
    return (p1.first < p2.first);
}

void ProjectiveSimulator::computeRankVec() {

    rankVec.clear();
    for(int i = 0; i < clipLayers->size() - 1; i++) {

        std::shared_ptr<set<std::shared_ptr<Clip>, clip_compare>> currLevel = clipLayers->at(i);
        for(std::shared_ptr<Clip> currClip : *currLevel) {
            rankVec.push_back(pair<double, std::shared_ptr<Clip>>(currClip->computeRank(), currClip));
        }
    }
    std::sort(rankVec.begin(), rankVec.end(), compareRanks);
    /*
    cout << "======================" << endl << "current ranking vec" << endl << "======================" << endl;
    for(int i = 0; i < rankVec.size(); ++i) {
        cout << *rankVec.at(i).second << " with rank " << rankVec.at(i).first << endl;
    }
    */

    if(PS_PRINT_RANKING_DEBUG_INFO)
        printRankVec();

}

int ProjectiveSimulator::getClipCount() {
    return rankVec.size();
}

void ProjectiveSimulator::printRankVec() {

    /*
    cout << "start printing rank vec" << endl;

    int prevRank = -1;
    for(std::pair<double, Clip*> p : rankVec) {
        if(prevRank != p.first) {
            cout << endl << "================================" << endl << p.first << endl << "================================" << endl;
            prevRank = p.first;
        }
        cout << *(p.second) << endl;
    }

    cout << "end printing rank vec" << endl;
    */

    cout << rankVec.size() << endl;

}

bool ProjectiveSimulator::fileExists(const std::string filePath) {
    ifstream f(filePath.c_str());
    if (f.good()) {
        f.close();
        return true;
    } else {
        f.close();
        return false;
    }
}

void ProjectiveSimulator::storePS(std::string targetFile) {

    int deleteFile = 0;
    if(fileExists(targetFile)) {
        cout << "(ProjectiveSimulator) file already exists. do you want to overwrite it? (0 = no / 1 = yes)" << endl;
        cin >> deleteFile;
        if(deleteFile == 1) {
            // do nothing and overwrite it later
        } else {
            cout << "(ProjectiveSimulator) no overwriting selected. skip storing" << endl;
            return;
        }
    }

    ofstream outFile;
    outFile.open(targetFile, ios::trunc);

    outFile << "V1.0" << endl;
    outFile << "general properties" << endl;
    outFile << "operationMode=" << operationMode << endl;
    outFile << "ranking=" << useRanking << endl;
    outFile << "gamma=" << gamma << endl;
    outFile << "maxNumberOfClips=" << maxNumberOfClips << endl;
    outFile << "immunityThresh=" << immunityThresh << endl;
    outFile << "levels=" << clipLayers->size() << endl;
    outFile << endl;

    outFile << "clips" << endl;
    for(std::shared_ptr<std::set<std::shared_ptr<Clip>, clip_compare>> layer : *clipLayers) {
        if(layer->size() > 0) {
            int currentLevel = (*layer->begin())->getLevel();
            outFile << "layer=" << currentLevel << endl;
            for(std::shared_ptr<Clip> cClip : *layer) {
                outFile << cClip->getIdVecString() << ";" << *cClip << ";" << cClip->getCurrentImmunity();
                if(currentLevel == 0) {
                    shared_ptr<PerceptClip> cpc = dynamic_pointer_cast<PerceptClip>(cClip);
                    outFile << ";" << cpc->getPerceptId();
                } else if(currentLevel == CLIP_H_LEVEL_FINAL) {
                    shared_ptr<ActionClip> cpc = dynamic_pointer_cast<ActionClip>(cClip);
                    outFile << ";" << cpc->getActionId();
                }
                outFile << endl;
            }
        }
    }
    outFile << endl;

    outFile << "connections" << endl;
    for(std::shared_ptr<std::set<std::shared_ptr<Clip>, clip_compare>> layer : *clipLayers) {
        if(layer->size() > 0) {
            int currentLevel = (*layer->begin())->getLevel();
            if(currentLevel != CLIP_H_LEVEL_FINAL) {
                outFile << "layer=" << currentLevel << endl;
                for(std::shared_ptr<Clip> cClip : *layer) {
                    int subClipCount = cClip->getSubClipCount();
                    outFile << cClip->getIdVecString() << ":" << endl;
                    for(int j = 0;  j < subClipCount; ++j) {
                        outFile << cClip->getSubClipByIdx(j)->getIdVecString() << ";" << cClip->getWeightByIdx(j) << endl;
                    }
                    outFile << endl;
                }
            }
        }
    }

}
