#ifndef PSEVALUATOR_H
#define PSEVALUATOR_H

#include <string>
#include <memory>
#include <fstream>
#include <iostream>
#include "../core/reward.h"
#include "../core/projectivesimulator.h"

class PSEvaluator {

public:

    PSEvaluator();

    static void produceStatistics(std::shared_ptr<ProjectiveSimulator> ps, std::shared_ptr<Reward> reward, int numberOfWalks, int clipImmunity, int rewardValue, std::ostream& outStream);

    static std::vector<double> evaluateStatistics(std::vector<std::shared_ptr<std::ifstream>>& inputStreams);
    static std::pair<std::vector<int>, std::vector<double>> evaluateStatistics(std::vector<std::string> inputFiles, std::vector<int> inputPos);

};

#endif // PSEVALUATOR_H
