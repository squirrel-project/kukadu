#ifndef KUKADU_PROJECTIVESIMULATOR_H
#define KUKADU_PROJECTIVESIMULATOR_H

#include <set>
#include <map>
#include <tuple>
#include <limits>
#include <vector>
#include <utility>
#include <algorithm>
#include <functional>
#include <kukadu/learning/projective_simulation/core/clip.hpp>
#include <kukadu/learning/projective_simulation/core/reward.hpp>
#include <kukadu/learning/projective_simulation/core/actionclip.hpp>
#include <kukadu/learning/projective_simulation/core/perceptclip.hpp>
#include <kukadu/types/kukadutypes.hpp>

namespace kukadu {

    struct clip_compare {

        bool operator() (KUKADU_SHARED_PTR<Clip> const& lhs, KUKADU_SHARED_PTR<Clip> const& rhs) {

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

        bool useRanking;
        bool doTraining;
        bool loadedFromFile;
        bool lastRunWasBored;

        int levels;
        int maxActionId;
        int maxPerceptId;
        int operationMode;
        int immunityThresh;
        int maxNumberOfClips;

        double gamma;

        std::string psFile;

        std::vector<double> boredomLevels;

        KUKADU_SHARED_PTR<Clip> predefinedFirstHop;
        KUKADU_SHARED_PTR<Clip> lastVisitedClip;

        int lastVisitedPreviousIdx;
        int lastVisitedLevel;
        bool lastBoredomResult;
        bool walkedFurtherSinceLastBoredom;

        std::vector<std::pair<double, KUKADU_SHARED_PTR<Clip> > > rankVec;

        KUKADU_SHARED_PTR<Reward> reward;
        KUKADU_SHARED_PTR<Clip> lastClipBeforeAction;
        KUKADU_SHARED_PTR<ActionClip> lastActionClip;
        KUKADU_SHARED_PTR<PerceptClip> lastPerceptClip;
        KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator;
        KUKADU_SHARED_PTR<PerceptClip> lastGeneralizedPercept;
        KUKADU_SHARED_PTR<std::vector<int> > intermediateHops;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > actionClips;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > perceptClips;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > > > clipLayers;

        kukadu_uniform_distribution intDist;

        void cleanByRank();
        void printRankVec();
        void computeRankVec();
        void construct(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, double gamma, int operationMode, bool useRanking);

        bool fileExists(const std::string filePath);

        int getIdVecLevel(KUKADU_SHARED_PTR<std::vector<int> > idVec);

        bool computeBoredom(KUKADU_SHARED_PTR<Clip> clip);

        void loadPsConstructor(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string file,
                               std::function<KUKADU_SHARED_PTR<Clip> (const std::string&, const int&, const int&, KUKADU_SHARED_PTR<kukadu_mersenne_twister>) > createClipFunc);

    public:

        ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string file);
        ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string file,
                            std::function<KUKADU_SHARED_PTR<Clip> (const std::string&, const int&, const int&, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator) > createClipFunc);
        ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, double gamma, int operationMode, bool useRanking);
        ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator,
                            KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > network,
                            double gamma, int operationMode, bool useRanking);
        ~ProjectiveSimulator();

        void printWeights();
        void setTrainingMode(bool doTraining);
        void setStandardImmunity(int immunity);
        void setBoredom(double boredom, int level);
        void setMaxNumberOfClips(int maxNumberOfClips);
        void connectNewClip(KUKADU_SHARED_PTR<Clip> conClip);
        void eliminateClip(KUKADU_SHARED_PTR<Clip> currClip);
        void generalize(KUKADU_SHARED_PTR<PerceptClip> nextClip);
        void fillClipLayersFromNetwork(KUKADU_SHARED_PTR<Clip> cl);

        KUKADU_SHARED_PTR<Clip> findClipByIdVec(KUKADU_SHARED_PTR<std::vector<int> > idVec);
        KUKADU_SHARED_PTR<Clip> findClipInLevelByIdVec(KUKADU_SHARED_PTR<std::vector<int> > idVec, int level);
        KUKADU_SHARED_PTR<Clip> findClipInLevelByLabel(std::string label, int level);

        bool compareIdVectors(std::vector<int>& idVec1, std::vector<int>& idVec2);

        int getClipCount();
        int getStandardImmunity();
        int generateNewActionId();
        int generateNewPerceptId();

        void addActionClip(KUKADU_SHARED_PTR<ActionClip> newAction);
        void addPerceptClip(KUKADU_SHARED_PTR<PerceptClip> newPercept);
        void setNextPredefinedPath(std::vector<KUKADU_SHARED_PTR<Clip> > hopPath);

        bool nextHopIsBored();
        bool lastHopWasBored();

        KUKADU_SHARED_PTR<Clip> getLastVisitedClip();

        std::tuple<bool, double, std::vector<int> > performRewarding();

        KUKADU_SHARED_PTR<std::vector<int> > getIntermediateHopIdx();

        std::pair<int, KUKADU_SHARED_PTR<Clip> > performRandomWalk(int untilLevel = PS_WALK_UNTIL_END, bool continueLastWalk = false);

        std::vector<KUKADU_SHARED_PTR<Clip> > retrieveClipsOnLayer(std::vector<int> queryId, int layer);

        // returns list of clips that have to be created
        KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > createNewClips(KUKADU_SHARED_PTR<PerceptClip> nextClip);
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > getPerceptClips();
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > getActionClips();
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > > > getClipLayers();

        std::vector<KUKADU_SHARED_PTR<Clip> > getClipsOnLayer(int layerId);

        void updatePsFile();
        void storePS(std::string targetFile);

        static constexpr auto IGNORE_ID = INT_MIN;

        static constexpr auto PS_USE_ORIGINAL = 1;
        static constexpr auto PS_USE_GEN = 2;

        static constexpr auto PS_PRINT_DEBUG_INFO = 0;
        static constexpr auto PS_PRINT_RANKING_DEBUG_INFO = 1;

        static constexpr auto PS_MAX_NUMBER_OF_CLIPS = 1000;

        static constexpr auto PS_WALK_UNTIL_END = -1;

    };

}

#endif
