#include "DictionaryGeneralizer.h"

using namespace std;
using namespace arma;

DictionaryGeneralizer::DictionaryGeneralizer(arma::vec timeCenters, arma::vec initQueryPoint, std::shared_ptr<ControlQueue> simulationQueue, std::shared_ptr<ControlQueue> executionQueue, std::string dictionaryPath,
                        double az, double bz, double stepSize,
                        double tolAbsErr, double tolRelErr, double ac, arma::vec trajMetricWeights,
                        double maxRelativeToMeanDistance, double as, double alpham) {

    dictTraj = std::shared_ptr<LinCombDmp>(new LinCombDmp(initQueryPoint.n_elem, dictionaryPath, az, bz, trajMetricWeights, timeCenters));

    this->simulationQueue = simulationQueue;
    this->executionQueue = executionQueue;
	this->stepSize = stepSize;
	this->tolAbsErr = tolAbsErr;
	this->tolRelErr = tolRelErr;
	
	this->as = 0.0;
	this->switchTime = 0.0;
	this->newQpSwitch = 1;
	
	this->currentQuery = initQueryPoint;
	this->ac = ac;
    this->as = as;
    this->alpham = alpham;
	
	this->maxRelativeToMeanDistance = maxRelativeToMeanDistance;

}

void DictionaryGeneralizer::setAs(double as) {
    this->as = as;
}

DictionaryGeneralizer::DictionaryGeneralizer(arma::vec timeCenters, arma::vec initQueryPoint, std::shared_ptr<ControlQueue> simulationQueue, std::shared_ptr<ControlQueue> executionQueue, std::string dictionaryPath, double az, double bz,
                  double stepSize, double tolAbsErr, double tolRelErr, double ac, double as, arma::mat metric, double maxRelativeToMeanDistance, double alpham) {

    dictTraj = std::shared_ptr<LinCombDmp>(new LinCombDmp(dictionaryPath, az, bz, metric, timeCenters));
    this->simulationQueue = simulationQueue;
    this->executionQueue = executionQueue;
	this->stepSize = stepSize;
	this->tolAbsErr = tolAbsErr;
	this->tolRelErr = tolRelErr;
	this->as = 0.0;
	this->switchTime = 0.0;
	this->newQpSwitch = 1;
    this->currentQuery = initQueryPoint;
	this->ac = ac;
	this->as = as;
    this->alpham = alpham;
    this->maxRelativeToMeanDistance = maxRelativeToMeanDistance;

}

void DictionaryGeneralizer::switchQueryPoint(vec query) {

    int points = getQueryPointCount();
    vec tmpNewCoefficients(points);

    int correspondingIdx = computeClosestT(currentTime, dictTraj->getTimeCenters());

    currentQuery = query;
    extendedQuery = computeExtendedQuery(currentTime, correspondingIdx, query);
    tmpNewCoefficients = computeNewCoefficients(dictTraj->getMetric().at(0), correspondingIdx, extendedQuery);
	
	switcherMutex.lock();

        currentQuery = query;
        switchTime = 0.0;
        newQpSwitch = 1;
        oldCoefficients = currentCoefficients;
        newCoefficients = tmpNewCoefficients;
        switchTime = 0;

	switcherMutex.unlock();
	
}

int DictionaryGeneralizer::getQueryPointCount() {
    return dictTraj->getQueryPoints().size();
}

QueryPoint DictionaryGeneralizer::getQueryPointByIndex(int idx) {
    return dictTraj->getQueryPoints().at(idx);
}

int DictionaryGeneralizer::getDegOfFreedom() {
	return dictTraj->getDegreesOfFreedom();
}

std::shared_ptr<ControllerResult> DictionaryGeneralizer::simulateTrajectory() {
    return executeGen(currentQuery, dictTraj->getTmax(), ac, as, 1);
}

// TODO: implement execute trajectory
std::shared_ptr<ControllerResult> DictionaryGeneralizer::executeTrajectory() {
    return executeGen(currentQuery, dictTraj->getTmax(), ac, as, 0);
}

void DictionaryGeneralizer::setTrajectory(std::shared_ptr<Trajectory> traj) {

    dictTraj = std::dynamic_pointer_cast<LinCombDmp>(dictTraj->copy());
    std::shared_ptr<LinCombDmp> castedTraj = std::dynamic_pointer_cast<LinCombDmp>(traj);
    vector<Mahalanobis> trajMetric = castedTraj->getMetric();
    dictTraj->setMetric(trajMetric);

}

double DictionaryGeneralizer::getCurrentTime() {
	return currentTime;
}

int DictionaryGeneralizer::computeClosestT(double t, arma::vec times) {

    double currentDist = abs(times(0) - t);
    int currentT = 0;

    for(int i = 0; i < times.n_elem; ++i) {
        if(currentDist > abs(times(i) - t)) {
            currentDist = abs(times(i) - t);
            currentT = i;
        }
    }

    return currentT;

}

arma::vec DictionaryGeneralizer::computeNewCoefficients(Mahalanobis metric, int correspondingIdx, arma::vec query) {

    int points = getQueryPointCount();
    vec distanceCoeffs(points);
    vec weightCoeffs(points);

    vec extendedCurrentDatabaseQuery(query.n_elem);

    // compute all distances
    for(int i = 0; i < points; ++i) {

        extendedCurrentDatabaseQuery.fill(0.0);
        extendedCurrentDatabaseQuery = computeExtendedQuery(currentTime, correspondingIdx, dictTraj->getQueryPoints().at(i).getQueryPoint());

        double currCoeff = metric.computeSquaredDistance(extendedCurrentDatabaseQuery, extendedQuery);
        distanceCoeffs(i) = currCoeff;

    }

    // normalize distances TODO: improve this
//    distanceCoeffs = distanceCoeffs / distanceCoeffs.max();

    // compute average distance
    double avgDist = 0.0;
    for(int i = 0; i < points; ++i)
        avgDist += distanceCoeffs(i);

    avgDist = avgDist / points;

    dictTraj->setCurrentQueryPoint(currentQuery);

    for(int i = 0; i < points; ++i) {
        double currCoeff = 1 / exp(alpham * distanceCoeffs(i));
        weightCoeffs(i) = currCoeff;
    }

    // drop trajectories that are too far away
    double tolerableDistance = avgDist * maxRelativeToMeanDistance;

    for(int i = 0; i < points; ++i) {
        double currCoeff = distanceCoeffs(i);
        if(currCoeff > tolerableDistance) {
            weightCoeffs(i) = 0.0;
        }
    }

    return weightCoeffs;

}

arma::vec DictionaryGeneralizer::computeExtendedQuery(double time, arma::vec query) {
    int correspondingIdx = computeClosestT(time, dictTraj->getTimeCenters());
    return computeExtendedQuery(time, correspondingIdx, query);
}

arma::vec DictionaryGeneralizer::computeExtendedQuery(double time, int correspondingIdx, arma::vec query) {

    vec timeCenters = dictTraj->getTimeCenters();
    int centersCount = timeCenters.n_elem;
    int querySize = query.n_elem;
    vec extendedQuery(centersCount * querySize);

    for(int i = 0; i < centersCount; ++i) {

        double currentQueryWeight = 0.0;

        if(i == correspondingIdx) {
            currentQueryWeight = 1.0;
        } else {
            currentQueryWeight = 0.0;
        }

        for(int j = 0; j < querySize; ++j) {
            extendedQuery(i * querySize + j) = query(j) * currentQueryWeight;
        }

    }

    return extendedQuery;

}

// TODO: find out why metric can produce negative distances after reinforcement learning
std::shared_ptr<ControllerResult> DictionaryGeneralizer::executeGen(arma::vec query, double tEnd, double ac, double as, int simulate) {
	
    double tStart = 0.0;
    int stepCount = (tEnd - tStart) / stepSize;
    int firstTime = 1;
	double norm = 0.0;
	currentTime = 0.0;
	int points = getQueryPointCount();
    int degOfFreedom = dictTraj->getDegreesOfFreedom();

    vec timeCenters = dictTraj->getTimeCenters();
    int centersCount = timeCenters.n_elem;
    int querySize = query.n_elem;

    vector<std::shared_ptr<DMPExecutor>> execs;
	currentQuery = query;
	newQpSwitch = 1;

	oldCoefficients = vec(points);
	newCoefficients = vec(points);
	currentCoefficients = vec(points);
	
    int isFirstIteration = 1;

    vector<vec> retY;
    for(int i = 0; i < degOfFreedom; ++i)
        retY.push_back(arma::vec(stepCount));

	vector<double> retT;

	// create all executors
	for(int i = 0; i < points; ++i) {

        QueryPoint currentQp = dictTraj->getQueryPoints().at(i);
        std::shared_ptr<DMPExecutor> currentExec = std::shared_ptr<DMPExecutor>(new DMPExecutor(currentQp.getDmp(), simulationQueue));
		currentExec->initializeIntegration(0, stepSize, tolAbsErr, tolRelErr);

		// if real execution use external error determination
        if(simulate) currentExec->useExternalError(1);

		execs.push_back(currentExec);

    }

    vector<Mahalanobis> metrics = dictTraj->getMetric();
    bool stopExecution = false;

    int extendedMetricSize = querySize * centersCount;
    mat extendedMetricMat(extendedMetricSize, extendedMetricSize);
    extendedMetricMat.fill(0.0);

    for(int i = 0; i < centersCount; ++i) {
        mat currentMetric = metrics.at(i).getM();
        for(int j = 0; j < querySize; ++j) {
            for(int k = 0; k < querySize; ++k) {
                extendedMetricMat(i * querySize + j, i * querySize + k) = currentMetric(j, k);
            }
        }
    }

    int correspondingIdx = -1;
    int oldCorrespondingIdx = -1;
    extendedQuery.fill(0.0);
    Mahalanobis extendedMetric(extendedMetricMat);

	// execute dmps and compute linear combination
    for(int j = 0; j < stepCount && !stopExecution; ++j, currentTime += stepSize) {

        oldCorrespondingIdx = correspondingIdx;
        correspondingIdx = computeClosestT(currentTime, timeCenters);

        if(oldCorrespondingIdx != correspondingIdx) {

            extendedQuery = computeExtendedQuery(currentTime, correspondingIdx, currentQuery);
            firstTime = true;

        }

		vec nextJoints(degOfFreedom);
		nextJoints.fill(0.0);

		// perform coefficient switching (mutex for the sake of thread safety)
		switcherMutex.lock();

			// if loop is executed first time, initialize everything
            if(firstTime) {

                currentCoefficients = computeNewCoefficients(extendedMetric, correspondingIdx, extendedQuery);
				oldCoefficients = newCoefficients = currentCoefficients;

				firstTime = 0;
				
            }

            if(dot(currentCoefficients, currentCoefficients) == 0.0) {
                string errStr = "(DictionaryGeneralizer) all coefficients are 0, please check your settings";
                cerr << errStr << endl;
                throw "(DictionaryGeneralizer) all coefficients are 0, please check your settings";
            }

			// compute coefficient switching
            double newCoeff = (1 - exp(- as * switchTime));
            double oldCoeff = exp(- as * switchTime);
            currentCoefficients =  newCoeff * newCoefficients + oldCoeff * oldCoefficients;
			
			// compute new normalization
			norm = 0.0;
			for(int i = 0; i < points; ++i) {
                double currCoeff = currentCoefficients(i);
                norm += currCoeff;
			}
			
			switchTime += stepSize;

        switcherMutex.unlock();

        // actual computation of trajectory using coefficients
        for(int i = 0; i < points; ++i) {

			vec currJoints(degOfFreedom);
			currJoints.fill(0.0);

            try {
                currJoints = execs.at(i)->doIntegrationStep(ac);
            } catch(const char* s) {
                puts(s);
                cerr << ": stopped execution at time " << currentTime << endl;
                stopExecution = true;
            //    return ret;
                break;
            }

            double currCoeff = currentCoefficients(i);
            nextJoints += currCoeff / norm * currJoints;

        }

        std::shared_ptr<ControlQueue> queue = NULL;
        if(simulate)
            queue = simulationQueue;
        else
            queue = executionQueue;

        // if real robot execution and first integration step --> move to initial position
        if(isFirstIteration) {

        //    cout << "(DictionaryGeneralizer) moving to initial execution position" << endl;
            queue->moveJoints(nextJoints);
            isFirstIteration = 0;
        //    cout << "(DictionaryGeneralizer) starting trajectory execution" << endl;

        } else {

            // synchronize to control queue (maximum one joint array has to be already in there --> needed for phase stopping such that DMPExecutor does not progress to fast)
            queue->synchronizeToControlQueue(0);
            queue->addJointsPosToQueue(nextJoints);

        }

		for(int i = 0; i < degOfFreedom; ++i)
            retY.at(i)(j) = nextJoints(i);

		retT.push_back(currentTime);

	}
	
	// clean up
    for(int i = 0; i < points; ++i)
        execs.at(i)->destroyIntegration();

    // should delete all stuff because of shared pointers
    execs.clear();

	currentTime = 0.0;

    return std::shared_ptr<ControllerResult>(new ControllerResult(stdToArmadilloVec(retT), retY));

}

std::shared_ptr<Trajectory> DictionaryGeneralizer::getTrajectory() {
    return dictTraj;
}

