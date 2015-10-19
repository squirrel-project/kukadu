#ifndef PROJECTIVESIMULATOR_H
#define PROJECTIVESIMULATOR_H

#include "clip.h"
#include "reward.h"
#include "actionclip.h"
#include "perceptclip.h"

#include <set>
#include <map>
#include <vector>
#include <random>
#include <utility>
#include <algorithm>
#include <limits>
#include <memory>

#define PS_PRINT_DEBUG_INFO 0
#define PS_PRINT_RANKING_DEBUG_INFO 0
#define PS_USE_ORIGINAL 1
#define PS_USE_GEN 2

#define PS_MAX_NUMBER_OF_CLIPS 1000

struct clip_compare {

    bool operator() (std::shared_ptr<Clip> const& lhs, std::shared_ptr<Clip> const& rhs) {

        /*
         *This object determines the order of the elements in the container:
         *it is a function pointer or a function object that takes two arguments of the same
         *type as the container elements, and returns true if the first argument is considered
         *to go before the second in the strict weak ordering it defines, and false otherwise.
         *
         *Two elements of a set are considered equivalent if key_comp returns false reflexively
         *(i.e., no matter the order in which the elements are passed as arguments)
        */

        // if same reference --> always the same
        if(lhs == rhs || *lhs == *rhs)
            return false;

        if(*lhs < *rhs)
            return true;

        return false;

    }

};

class ProjectiveSimulator {

private:

    double gamma;
    double boredom;

    int levels;
    int operationMode;
    int immunityThresh;
    int maxNumberOfClips;

    bool useRanking;
    bool useBoredom;

    std::shared_ptr<Reward> reward;

    std::shared_ptr<std::mt19937> generator;
    std::uniform_int_distribution<int> intDist;

    std::shared_ptr<Clip> lastClipBeforeAction;
    std::shared_ptr<ActionClip> lastActionClip;
    std::shared_ptr<PerceptClip> lastPerceptClip;
    std::shared_ptr<PerceptClip> lastGeneralizedPercept;

    std::shared_ptr<std::vector<int>> intermediateHops;
    std::vector<std::pair<double, std::shared_ptr<Clip>>> rankVec;
    std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> actionClips;
    std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>> perceptClips;

    // includes action clips
    std::shared_ptr<std::vector<std::shared_ptr<std::set<std::shared_ptr<Clip>, clip_compare>>>> clipLayers;

    void cleanByRank();
    void printRankVec();
    void computeRankVec();

    bool fileExists(const std::string filePath);

    int getIdVecLevel(std::shared_ptr<std::vector<int>> idVec);

    double computeBoredem(std::shared_ptr<Clip> clip);

    std::shared_ptr<Clip> findClipByIdVec(std::shared_ptr<std::vector<int>> idVec);
    std::shared_ptr<Clip> findClipInLevelByIdVec(std::shared_ptr<std::vector<int>> idVec, int level);

    void construct(std::shared_ptr<Reward> reward, std::shared_ptr<std::mt19937> generator, double gamma, int operationMode, bool useRanking);

public:

    ProjectiveSimulator(std::shared_ptr<Reward> reward, std::shared_ptr<std::mt19937> generator, std::string file);
    ProjectiveSimulator(std::shared_ptr<Reward> reward, std::shared_ptr<std::mt19937> generator, double gamma, int operationMode, bool useRanking);
    ProjectiveSimulator(std::shared_ptr<Reward> reward, std::shared_ptr<std::mt19937> generator,
                        std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>> network,
                        double gamma, int operationMode, bool useRanking);
    ~ProjectiveSimulator();

    void printWeights();
    void setBoredom(double boredom);
    void setStandardImmunity(int immunity);
    void setMaxNumberOfClips(int maxNumberOfClips);
    void connectNewClip(std::shared_ptr<Clip> conClip);
    void eliminateClip(std::shared_ptr<Clip> currClip);
    void generalize(std::shared_ptr<PerceptClip> nextClip);
    void fillClipLayersFromNetwork(std::shared_ptr<Clip> cl);

    int getClipCount();
    int getStandardImmunity();

    std::pair<bool, double> performRewarding();

    std::shared_ptr<std::vector<int>> getIntermediateHopIdx();

    std::shared_ptr<ActionClip> performRandomWalk();

    // returns list of clips that have to be created
    std::shared_ptr<std::set<std::shared_ptr<Clip>, clip_compare>> createNewClips(std::shared_ptr<PerceptClip> nextClip);
    std::shared_ptr<std::vector<std::shared_ptr<PerceptClip>>> getPerceptClips();
    std::shared_ptr<std::vector<std::shared_ptr<ActionClip>>> getActionClips();
    std::shared_ptr<std::vector<std::shared_ptr<std::set<std::shared_ptr<Clip>, clip_compare>>>> getClipLayers();

    void storePS(std::string targetFile);

};

#endif // PROJECTIVESIMULATOR_H
