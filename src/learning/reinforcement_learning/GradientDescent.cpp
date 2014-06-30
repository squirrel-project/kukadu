#include "GradientDescent.h"

using namespace arma;
using namespace std;

//bool rewardComparator(pair <double, Trajectory*> i, pair <double, Trajectory*> j) { return (i.first > j.first); }

GradientDescent::GradientDescent(TrajectoryExecutor* trajEx, std::vector<Trajectory*> initDmp, double explorationSigma, int updatesPerRollout, int importanceSamplingCount, CostComputer* cost, ControlQueue* movementQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) : GeneralReinforcer(trajEx, cost, movementQueue) {

    vector<double> intSigmas;

    // init sampler
    for(int i = 0; i < initDmp.at(0)->getCoefficients().at(0).n_elem; ++i) {
        vec currCoeff = initDmp.at(0)->getCoefficients().at(0);
        normal_distribution<double> normal(0, explorationSigma);
        normals.push_back(normal);
        intSigmas.push_back(abs(explorationSigma));
    }

    construct(initDmp, intSigmas, updatesPerRollout, importanceSamplingCount, cost, movementQueue, ac, dmpStepSize, tolAbsErr, tolRelErr);

}

GradientDescent::GradientDescent(TrajectoryExecutor* trajEx, std::vector<Trajectory*> initDmp, vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, CostComputer* cost, ControlQueue* movementQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) : GeneralReinforcer(trajEx, cost, movementQueue) {

    // init sampler
    for(int i = 0; i < initDmp.at(0)->getCoefficients().at(0).n_elem; ++i) {

        vec currCoeff = initDmp.at(0)->getCoefficients().at(0);
        normal_distribution<double> normal(0, explorationSigmas.at(i));
        normals.push_back(normal);

    }

    construct(initDmp, explorationSigmas, updatesPerRollout, importanceSamplingCount, cost, movementQueue, ac, dmpStepSize, tolAbsErr, tolRelErr);

}

void GradientDescent::construct(std::vector<Trajectory*> initDmp, vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, CostComputer* cost, ControlQueue* movementQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) {

    setLastUpdate(initDmp.at(0));

    this->initDmp = initDmp;
    this->explorationSigma = explorationSigma;
    this->sigmas = explorationSigmas;
    this->updatesPerRollout = updatesPerRollout;
    this->importanceSamplingCount = importanceSamplingCount;

}

std::vector<Trajectory*> GradientDescent::getInitialRollout() {
    vector<Trajectory*> ret;
    ret.push_back(initDmp.at(0));
    return ret;
}

std::vector<Trajectory*> GradientDescent::computeRolloutParamters() {

    Trajectory* lastUp = getLastUpdate();
    vector<vec> dmpCoeffs = lastUp->getCoefficients();
    vector<Trajectory*> nextCoeffs;

    for(int k = 0; k < updatesPerRollout; ++k) {

        for(int i = 0; i < dmpCoeffs.size(); ++i) {

            vec currCoeff = dmpCoeffs.at(i);

            for(int j = 0; j < currCoeff.n_elem; ++j) {

                normal_distribution<double> normal = normals.at(j);
                double eps = normal(generator);

                currCoeff(j) += eps;

            }

            dmpCoeffs[i] = currCoeff;

        }

        Trajectory* nextUp = lastUp->copy();
        nextUp->setCoefficients(dmpCoeffs);

        nextCoeffs.push_back(nextUp);

    }

    return nextCoeffs;

}

Trajectory* GradientDescent::updateStep() {
    /*

    Trajectory* lastUp = getLastUpdate();

    vector<Trajectory*> lastDmps = getLastRolloutParameters();
    vector<double> lastRewards = getLastRolloutCost();

    // add rollouts to history
    for(int i = 0; i < lastRewards.size(); ++i) {
        pair <double, Trajectory*> p(lastRewards.at(i), lastDmps.at(i));
        sampleHistory.push_back(p);
    }

    // sort by reward...
    sort(sampleHistory.begin(), sampleHistory.end(), rewardComparator);

    // ...and discard bad samples
    int histSize = sampleHistory.size();
    for(int i = (importanceSamplingCount - 1); i < histSize; ++i)
        sampleHistory.pop_back();

    double totalReward = 0.0;
    for(int i = 0; i < sampleHistory.size(); ++i)
        totalReward = totalReward + sampleHistory.at(i).first;

    vector<vec> lastUpCoeffs = lastUp->getCoefficients();
    vec newCoeffsJ(lastUpCoeffs.at(0).n_elem);
    vector<vec> newCoeffs;

    for(int i = 0; i < lastUp->getCoefficients().size(); ++i) {
        vec v = lastUp->getCoefficients().at(i);
        newCoeffs.push_back(v);
    }

    // for each degree of freedom
    for(int i = 0; i < newCoeffs.size(); ++i) {

        vec currentDegCoeffs = newCoeffs.at(i);

        // go through all samples and weight rollouts
        for(int j = 0; j < sampleHistory.size(); ++j) {
            currentDegCoeffs += sampleHistory.at(j).first / totalReward * (sampleHistory.at(j).second->getCoefficients().at(i) - lastUpCoeffs.at(i));
        }

        newCoeffs[i] = currentDegCoeffs;

    }

    //Dmp newUp(lastUp);
    Trajectory* newUp = lastUp->copy();
    newUp->setCoefficients(newCoeffs);

//	cout << newCoeffs.at(0).t() << endl;

    return newUp;
*/
}
