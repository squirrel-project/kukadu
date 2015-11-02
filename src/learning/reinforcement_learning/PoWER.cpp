#include "PoWER.h"

using namespace arma;
using namespace std;

bool rewardComparator (pair <double, KUKADU_SHARED_PTR<Trajectory> > i, pair <double, KUKADU_SHARED_PTR<Trajectory> > j) { return (i.first > j.first); }

PoWER::PoWER(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, double explorationSigma, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) : GeneralReinforcer(trajEx, cost, simulationQueue, executionQueue) {
	
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = std::default_random_engine(seed);

	vector<double> intSigmas;
	
	// init sampler
	for(int i = 0; i < initDmp.at(0)->getCoefficients().at(0).n_elem; ++i) {
		normal_distribution<double> normal(0, explorationSigma);
		normals.push_back(normal);
		intSigmas.push_back(abs(explorationSigma));
	}
	
    construct(initDmp, intSigmas, updatesPerRollout, importanceSamplingCount, cost, simulationQueue, executionQueue, ac, dmpStepSize, tolAbsErr, tolRelErr);
	
}

PoWER::PoWER(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) : GeneralReinforcer(trajEx, cost, simulationQueue, executionQueue) {

	// init sampler

    if(explorationSigmas.size() < initDmp.at(0)->getCoefficients().at(0).n_elem) {
        cerr << "you defined too less exploration sigmas" << endl;
        throw "you defined too less exploration sigmas";
    }

	for(int i = 0; i < initDmp.at(0)->getCoefficients().at(0).n_elem; ++i) {

		normal_distribution<double> normal(0, explorationSigmas.at(i));
		normals.push_back(normal);

	}

    construct(initDmp, explorationSigmas, updatesPerRollout, importanceSamplingCount, cost, simulationQueue, executionQueue, ac, dmpStepSize, tolAbsErr, tolRelErr);

}

void PoWER::construct(std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) {
	
	setLastUpdate(initDmp.at(0));
	
	this->initDmp = initDmp;
	this->explorationSigma = explorationSigma;
	this->sigmas = explorationSigmas;
	this->updatesPerRollout = updatesPerRollout;
	this->importanceSamplingCount = importanceSamplingCount;
	
}

std::vector<KUKADU_SHARED_PTR<Trajectory> > PoWER::getInitialRollout() {
    vector<KUKADU_SHARED_PTR<Trajectory> > ret;
	ret.push_back(initDmp.at(0));
	return ret;
}

std::vector<KUKADU_SHARED_PTR<Trajectory> > PoWER::computeRolloutParamters() {
	
    KUKADU_SHARED_PTR<Trajectory> lastUp = getLastUpdate();
	vector<vec> dmpCoeffs = lastUp->getCoefficients();
    vector<KUKADU_SHARED_PTR<Trajectory> > nextCoeffs;
	
	for(int k = 0; k < updatesPerRollout; ++k) {
	
        vec currCoeff;
		for(int i = 0; i < dmpCoeffs.size(); ++i) {
			
            currCoeff = dmpCoeffs.at(i);
			
			for(int j = 0; j < currCoeff.n_elem; ++j) {
			
				normal_distribution<double> normal = normals.at(j);
				double eps = normal(generator);
				
				currCoeff(j) += eps;
				
			}
			
			dmpCoeffs[i] = currCoeff;
			
		}

        //TODO: blew1 und blew2 do not deliver same result if there are more time centers (maybe not decompose whole extended M but single submatrices)

//        cout << "blew1: " << dmpCoeffs.at(0).t() << endl;
        KUKADU_SHARED_PTR<Trajectory> nextUp = lastUp->copy();
        nextUp->setCoefficients(dmpCoeffs);
//        cout << "blew2: " << nextUp->getCoefficients().at(0).t() << endl;
		
		nextCoeffs.push_back(nextUp);

	}
	
	return nextCoeffs;
	
}

KUKADU_SHARED_PTR<Trajectory> PoWER::updateStep() {

    KUKADU_SHARED_PTR<Trajectory> lastUp = getLastUpdate();

    vector<KUKADU_SHARED_PTR<Trajectory> > lastDmps = getLastRolloutParameters();
	vector<double> lastRewards = getLastRolloutCost();

	// add rollouts to history
	for(int i = 0; i < lastRewards.size(); ++i) {
        pair <double, KUKADU_SHARED_PTR<Trajectory> > p(lastRewards.at(i), lastDmps.at(i));
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
    cout << "====================" << endl;
	
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
    KUKADU_SHARED_PTR<Trajectory> newUp = lastUp->copy();
	newUp->setCoefficients(newCoeffs);

//	cout << newCoeffs.at(0).t() << endl;
	
	return newUp;

}

