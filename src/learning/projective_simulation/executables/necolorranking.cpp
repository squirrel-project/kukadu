#include <ctime>
#include <chrono>
#include <vector>
#include <numeric>
#include <random>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <stdexcept>
#include <execinfo.h>
#include <sstream>

#include "../core/clip.h"
#include "../core/actionclip.h"
#include "../core/perceptclip.h"
#include "../core/psevaluator.h"
#include "../core/projectivesimulator.h"

#include "../visualization/treedrawer.h"
#include "../utils/Tokenizer.h"

#include "../application/neverendingcolorreward.h"

#include <ctime>
#include <chrono>
#include <vector>
#include <numeric>
#include <random>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <stdexcept>
#include <execinfo.h>
#include <sstream>
#include <getopt.h>

#define ASY_WR_CONVERGENCE_TOL 0.01
#define ASY_WR_NUMBER_OF_AGENTS 2000
#define ASY_WR_NUMBER_OF_WALKS 100000
#define ASY_WR_NUMBER_OF_ACTIONS 2
#define ASY_WR_GAMMA 0.0
#define ASY_WR_MAX_NUMBER_OF_CLIPS 300
#define ASY_WR_CLIP_IMMUNITY 60
#define ASY_WR_NUMBER_OF_CATS 10
#define ASY_WR_START_NUMER_OF_CATS 2

using namespace std;

int main(int argc, char** args) {

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    int numberOfWalks = ASY_WR_NUMBER_OF_WALKS;
    int startNumberOfCats = ASY_WR_START_NUMER_OF_CATS;
    int maxNumberOfCats = ASY_WR_NUMBER_OF_CATS;
    int clipImmunity = ASY_WR_CLIP_IMMUNITY;
    int maxClipNumber = ASY_WR_MAX_NUMBER_OF_CLIPS;

    string outFilePath = "./necolornoranking.txt";

    char c = 0;
    opterr = 1;
    while((c = getopt(argc, args, "c:C:W:o:S:I:M:")) != -1) {
        switch(c) {
        case 'c':
            startNumberOfCats = atoi(optarg);
            break;
        case 'C':
            maxNumberOfCats = atoi(optarg);
            break;
        case 'W':
            numberOfWalks = atoi(optarg);
            break;
        case 'o':
            outFilePath = string(optarg);
            break;
        case 'S':
            seed = atoi(optarg);
            break;
        case 'I':
            clipImmunity = atoi(optarg);
            break;
        case 'M':
            maxClipNumber = atoi(optarg);
            break;
        }
    }

    cout << "options parsed" << endl;

    cout << "summary" << endl << "======================================" << endl;
    cout << "startNumberOfCats: " << startNumberOfCats << endl;
    cout << "maxNumberOfCats: " << maxNumberOfCats << endl;
    cout << "numberOfWalks: " << numberOfWalks << endl;
    cout << "outFilePath: " << outFilePath << endl;
    cout << "randomSeed: " << seed << endl;
    cout << "clipImmunity: " << clipImmunity << endl;
    cout << "maxClipNumber: " << maxClipNumber << endl;
    cout << "======================================" << endl << endl;

    std::shared_ptr<std::mt19937> generator = std::shared_ptr<std::mt19937>(new std::mt19937(seed));

    ofstream outFile;
    outFile.open(outFilePath);

    outFile << "program mode is neverending color scenario without ranking" << endl;

    for(int numberOfCats = startNumberOfCats; numberOfCats <= maxNumberOfCats; ++numberOfCats) {

        outFile << endl << "number of cats: " << numberOfCats << endl;

        vector<double> asymptoticRewards;
        for(int i = 0; i < numberOfWalks; ++i)
            asymptoticRewards.push_back(0.0);

        std::shared_ptr<NeverendingColorReward> trafficReward = nullptr;
        std::shared_ptr<ProjectiveSimulator> currentProjSim = nullptr;

        outFile << "agent number " << 0 << " with " << numberOfCats << " cats" << endl;

        trafficReward = std::shared_ptr<NeverendingColorReward>(new NeverendingColorReward(generator, ASY_WR_NUMBER_OF_ACTIONS, numberOfCats, true));
        currentProjSim = std::shared_ptr<ProjectiveSimulator>(new ProjectiveSimulator(trafficReward, generator, ASY_WR_GAMMA, PS_USE_GEN, true));

        currentProjSim->setStandardImmunity(clipImmunity);
        currentProjSim->setMaxNumberOfClips(maxClipNumber);

        PSEvaluator::produceStatistics(currentProjSim, trafficReward, numberOfWalks, clipImmunity, NEVERENDINGCOLORREWARD_SUCCESSFUL_REWARD, outFile);

        currentProjSim = nullptr;
        trafficReward = nullptr;


    }

    return EXIT_SUCCESS;

}
