#include "clip.h"
#include "../utils/Tokenizer.h"
#include <typeinfo>

using namespace std;

Clip::Clip(int level, std::shared_ptr<std::mt19937> generator, std::string clipValues, int immunity) {

    construct(level, generator, Clip::getIdVectorFromString(clipValues), immunity);

}

Clip::Clip(int level, std::shared_ptr<std::mt19937> generator, std::shared_ptr<std::vector<int>> clipDimensionValues, int immunity) {

    construct(level, generator, clipDimensionValues, immunity);

}

void Clip::construct(int level, std::shared_ptr<std::mt19937> generator, std::shared_ptr<std::vector<int>> clipValues, int immunity) {

    this->level = level;
    this->generator = generator;
    this->clipDimensionValues = clipValues;
    this->subClips = std::shared_ptr<std::vector<std::shared_ptr<Clip>>>(new std::vector<std::shared_ptr<Clip>>());
    this->parents = std::shared_ptr<std::set<std::shared_ptr<Clip>>>(new std::set<std::shared_ptr<Clip>>());
    this->subClipsSet = std::shared_ptr<std::set<std::shared_ptr<Clip>>>(new std::set<std::shared_ptr<Clip>>());
    this->visitedSubNode = CLIP_H_NOT_WALKED_YET;
    this->previousRank = -1;
    this->initialImmunity = this->immunity = immunity;
    this->gotDeleted = 0;

}

std::shared_ptr<std::vector<int>> Clip::getIdVectorFromString(std::string str) {

    std::shared_ptr<std::vector<int>> retVec = std::shared_ptr<std::vector<int>>(new std::vector<int>());
    Tokenizer tok(str, "(),");

    string t = "";

    while((t = tok.next()).compare(""))
        retVec->push_back(atoi(t.c_str()));

    return retVec;

}

Clip::~Clip() {
    subClips = nullptr;
    parents = nullptr;
    subClipsSet = nullptr;
}

double Clip::computeSubEntropy() const {

    double entropy = 0.0;
    vector<double> probs = discDist.probabilities();

    for(double prob : probs)
        entropy -= prob * log2(prob);

    return entropy;

}

void Clip::setImmunity(int immunity) {
    this->immunity = immunity;
}

bool Clip::isImmune() {
    return immunity;
}

int Clip::getCurrentImmunity() {
    return immunity;
}

void Clip::decreaseImmunity() {
    if(immunity > 0)
        --immunity;
}

void Clip::addParent(std::shared_ptr<Clip> par) {
    parents->insert(par);
}

void Clip::addParentDownwards(std::shared_ptr<Clip> par) {
    addParent(par);
    par->addSubClip(shared_from_this(), CLIP_H_STD_WEIGHT);
    for(std::shared_ptr<Clip> child : *subClips) {
        child->addParentDownwards(par);
    }
}

void Clip::addChildUpwards(std::shared_ptr<Clip> sub) {
    this->addSubClip(sub, CLIP_H_STD_WEIGHT);
    for(std::shared_ptr<Clip> parent : *parents)
        parent->addChildUpwards(sub);
}

std::pair<int, std::shared_ptr<Clip>> Clip::jumpNextRandom() {

    visitedSubNode = discDist(*generator);
    return pair<int, std::shared_ptr<Clip>>(visitedSubNode, subClips->at(visitedSubNode));

}

void Clip::initRandomGenerator() {

    discDist = discrete_distribution<int>(subH.begin(), subH.end());

}

void Clip::addSubClip(std::shared_ptr<Clip> sub, int weight) {

    pair<set<std::shared_ptr<Clip>>::iterator, bool> inserted = subClipsSet->insert(sub);
    if(inserted.second) {
        subClips->push_back(sub);
        subH.push_back(weight);
        sub->addParent(shared_from_this());
        initRandomGenerator();
    }

}

void Clip::setChildren(std::shared_ptr<std::vector<std::shared_ptr<Clip>>> children) {

    int childCount = children->size();
    std::vector<double> newH;

    for(int i = 0; i < childCount; ++i) {
        children->at(i)->addParent(shared_from_this());
        newH.push_back(CLIP_H_STD_WEIGHT);
    }

    setChildren(children, newH);

}

void Clip::setChildren(std::shared_ptr<std::vector<std::shared_ptr<Clip> > > children, std::vector<double> weights) {

    subClips = nullptr;
    this->subClips = children;
    this->subH = weights;
    initRandomGenerator();

}

void Clip::addHAndInitialize(int idx, int addWeight) {

    subH.at(idx) = addWeight;
    initRandomGenerator();

}

int Clip::getSubClipCount() {
    return subH.size();

}

void Clip::setPreviousRank() {
    previousRank = computeRank();
}

int Clip::getPreviousRank() {
    return previousRank;
}

// sum outgoing weights - initial weights * number of outgoing edges
double Clip::computeRank() const {

    double outWeights = std::accumulate(subH.begin(), subH.end(), 0.0);
    double outNum = subH.size();

    return outWeights - CLIP_H_STD_WEIGHT * outNum;

}

void Clip::updateWeights(double reward, double gamma) {

    int subClipCount = subH.size();
    for(int i = 0; i < subClipCount; ++i) {

        double currentWeight = subH.at(i);
        if(visitedSubNode != CLIP_H_NOT_WALKED_YET && i == visitedSubNode)
            subH.at(i) = max(1.0, currentWeight - gamma * (currentWeight - 1) + reward);
        else
            subH.at(i) = currentWeight - gamma * (currentWeight - 1);

    }
    visitedSubNode = CLIP_H_NOT_WALKED_YET;
    initRandomGenerator();

}

void Clip::printSubWeights() {
    for(int i = 0; i < subH.size(); ++i) {
        cout << subH.at(i) << " ";
    }
    cout << endl;
}

bool Clip::compareIdVecs(const std::shared_ptr<std::vector<int> > vec1, const std::shared_ptr<std::vector<int> > vec2) {

    int s1 = vec1->size();
    if(s1 == vec2->size()) {
        for(int i = 0; i < s1; ++i)
            if(vec1->at(i) != vec2->at(i))
                return false;
        return true;
    }
    throw "(Clip==) clip dimensions do not match";

}

bool operator== (const Clip &o1, const Clip &o2) {

    return Clip::compareIdVecs(o1.getClipDimensions(), o2.getClipDimensions());

}

bool operator!= (const Clip &o1, const Clip &o2) {
    return !(o1 == o2);
}

bool operator< (const Clip &o1, const Clip &o2) {

    int s1 = o1.clipDimensionValues->size();
    if(s1 == o2.clipDimensionValues->size()) {
        for(int i = 0; i < s1; ++i)
            if(o1.clipDimensionValues->at(i) == o2.clipDimensionValues->at(i)) {}
            else return o1.clipDimensionValues->at(i) < o2.clipDimensionValues->at(i);
    }
    return false;
    throw "(Clip==) clip dimensions do not match";

}

bool Clip::isCompatibleSubclip(std::shared_ptr<Clip> c) {

    // if c is action clip --> its always a compatible subclip
    if(c->getLevel() == CLIP_H_LEVEL_FINAL)
        return true;
    else if(this->getLevel() == CLIP_H_LEVEL_FINAL)
        return false;

    std::shared_ptr<std::vector<int>> cClips = c->clipDimensionValues;
    int clipSize = cClips->size();
    if(this->clipDimensionValues->size() == clipSize) {
        for(int i = 0; i < cClips->size(); ++i) {
            if(this->clipDimensionValues->at(i) != cClips->at(i) && cClips->at(i) != CLIP_H_HASH_VAL)
                return false;
        }
        return true;
    }
    throw "(Clip checkCompatible) clip dimensions do not match";

}

void Clip::setClipDimensionValues(std::shared_ptr<std::vector<int>> vals) {
    this->clipDimensionValues = vals;
}

std::shared_ptr<Clip> Clip::getSubClipByIdx(int idx) {
    return subClips->at(idx);
}

double Clip::getWeightByIdx(int idx) {
    return subH.at(idx);
}

string Clip::toString() const {

    stringstream ss;
    ss << "(";
    for(int dim : *clipDimensionValues)
        if(dim != CLIP_H_HASH_VAL)
            ss << dim << ", ";
        else
            ss << "#, ";
    string retStr = ss.str();
    return retStr.substr(0, retStr.length() - 2) + ")";

}

std::ostream& operator<<(std::ostream &strm, const Clip &c) {
    return strm << c.toString();
}

std::shared_ptr<std::vector<int> > Clip::getClipDimensions() const {
    return clipDimensionValues;
}

std::shared_ptr<std::set<std::shared_ptr<Clip>>> Clip::getParents() {
    return parents;
}

void Clip::removeAllSubClips() {
    vector<std::shared_ptr<Clip>> subClipCopy(*subClips);
    for(std::shared_ptr<Clip> subClip : subClipCopy) {
        removeSubClipWoRand(subClip);
    }
    gotDeleted = 1;
}

void Clip::removeParentClip(std::shared_ptr<Clip> c) {
    parents->erase(c);
}

void Clip::removeSubClip(std::shared_ptr<Clip> clip) {

    removeSubClipWoRand(clip);
    initRandomGenerator();

}

void Clip::removeSubClipWoRand(std::shared_ptr<Clip> clip) {

    subClipsSet->erase(clip);
    vector<std::shared_ptr<Clip>>::iterator foundIt = std::find(subClips->begin(), subClips->end() + 1, clip);
    int elIdx = foundIt - subClips->begin();
    subClips->erase(foundIt);
    subH.erase(subH.begin() + elIdx);
    clip->removeParentClip(shared_from_this());

}

int Clip::getInitialImmunity() {
    return initialImmunity;
}

std::shared_ptr<Clip> Clip::compareClip(std::shared_ptr<Clip> c) {

    std::shared_ptr<vector<int>> cVec = c->clipDimensionValues;
    int cVecSize = cVec->size();
    int thisVecSize = clipDimensionValues->size();
    if(cVecSize != thisVecSize)
        return NULL;

    int hashCount = 0;
    std::shared_ptr<vector<int>> clipValues = std::shared_ptr<vector<int>>(new vector<int>());
    for(int i = 0, val = 0; i < cVecSize; ++i) {
        if((val = cVec->at(i)) == clipDimensionValues->at(i) && cVec->at(i) != CLIP_H_HASH_VAL)
            clipValues->push_back(val);
        else {
            clipValues->push_back(CLIP_H_HASH_VAL);
            ++hashCount;
        }
    }

    // level is given by number of hashes
    if(hashCount)
        return std::shared_ptr<Clip>(new Clip(hashCount, generator, clipValues, this->getInitialImmunity()));
    else {
        // with shared pointers it is important that this line is never executed --> return shared_from_this instead
        // return std::shared_ptr<Clip>(this);
        return shared_from_this();
    }

}

int Clip::getDimensionality() {
    return clipDimensionValues->size();
}

int Clip::getLevel() {
    return level;
}

std::string Clip::getIdVecString() const {

    stringstream s;
    s << "(";

    bool isFirst = true;
    for(int val : *clipDimensionValues) {

        if(isFirst)
            isFirst = false;
        else
            s << ", ";

        s << val;
    }
    s << ")";

    return s.str();

}
