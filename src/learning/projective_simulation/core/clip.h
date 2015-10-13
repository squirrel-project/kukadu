#ifndef CLIP_H
#define CLIP_H

#include <iostream>
#include <random>
#include <vector>
#include <utility>
#include <limits>
#include <string>
#include <set>
#include <limits.h>
#include <algorithm>
#include <sstream>
#include <memory>
#include <string>
#include <vector>

// start weight according to the paper
#define CLIP_H_STD_WEIGHT 1
#define CLIP_H_HASH_VAL INT_MIN
#define CLIP_H_LEVEL_FINAL -1
#define CLIP_H_NOT_WALKED_YET -1

#define PS_DEFAULT_IMMUNITY 1000

class Clip : public std::enable_shared_from_this<Clip> {

private:

    int level;
    int immunity;
    int gotDeleted;
    int previousRank;
    int initialImmunity;

    std::shared_ptr<std::mt19937> generator;

    std::vector<double> subH;

    std::discrete_distribution<int> discDist;

    std::shared_ptr<std::set<std::shared_ptr<Clip>>> parents;
    std::shared_ptr<std::set<std::shared_ptr<Clip>>> subClipsSet;

    std::shared_ptr<std::vector<int>> clipDimensionValues;
    std::shared_ptr<std::vector<std::shared_ptr<Clip>>> subClips;

    void construct(int level, std::shared_ptr<std::mt19937> generator, std::shared_ptr<std::vector<int>> clipValues, int immunity);

protected:

    int visitedSubNode;

public:

    Clip(int level, std::shared_ptr<std::mt19937> generator, std::string clipValues, int immunity);
    Clip(int level, std::shared_ptr<std::mt19937> generator, std::shared_ptr<std::vector<int>> clipValues, int immunity);
    ~Clip();

    // must set the visitedSubnode member if overwritten --> otherwise it will not update its weights
    // todo: remove that requirement that subclasses have to set that by themselves
    virtual std::pair<int, std::shared_ptr<Clip> > jumpNextRandom();

    static std::shared_ptr<std::vector<int>> getIdVectorFromString(std::string str);
    static bool compareIdVecs(const std::shared_ptr<std::vector<int> > vec1, const std::shared_ptr<std::vector<int> > vec2);

    void printSubWeights();
    void setPreviousRank();
    void decreaseImmunity();
    void removeAllSubClips();
    void initRandomGenerator();
    void setImmunity(int immunity);
    void addParent(std::shared_ptr<Clip> par);
    void removeSubClip(std::shared_ptr<Clip> clip);
    void removeParentClip(std::shared_ptr<Clip> c);
    void addHAndInitialize(int idx, int addWeight);
    void updateWeights(double reward, double gamma);
    void addChildUpwards(std::shared_ptr<Clip> sub);
    void addParentDownwards(std::shared_ptr<Clip> par);
    void removeSubClipWoRand(std::shared_ptr<Clip> clip);
    void addSubClip(std::shared_ptr<Clip> sub, int weight);
    void setClipDimensionValues(std::shared_ptr<std::vector<int> > vals);
    void setChildren(std::shared_ptr<std::vector<std::shared_ptr<Clip>>> children);
    void setChildren(std::shared_ptr<std::vector<std::shared_ptr<Clip>>> children, std::vector<double> weights);

    bool isImmune();
    virtual bool isCompatibleSubclip(std::shared_ptr<Clip> c);

    int getLevel();
    int getPreviousRank();
    int getSubClipCount();
    int getDimensionality();
    int getInitialImmunity();
    int getCurrentImmunity();

    double getWeightByIdx(int idx);
    virtual double computeRank() const;

    std::string getIdVecString() const;
    virtual std::string toString() const;

    std::shared_ptr<Clip> getSubClipByIdx(int idx);
    std::shared_ptr<Clip> compareClip(std::shared_ptr<Clip> c);

    std::shared_ptr<std::vector<int>> getClipDimensions() const;
    std::shared_ptr<std::set<std::shared_ptr<Clip>>> getParents();

    friend bool operator< (const Clip &o1, const Clip &o2);
    friend bool operator== (const Clip &o1, const Clip &o2);
    friend bool operator!= (const Clip &o1, const Clip &o2);

    friend std::ostream& operator<<(std::ostream &strm, const Clip &c);

};

#endif // CLIP_H
