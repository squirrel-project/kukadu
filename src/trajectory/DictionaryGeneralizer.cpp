#include "DictionaryGeneralizer.h"

using namespace std;
using namespace arma;

DictionaryGeneralizer::DictionaryGeneralizer(arma::vec timeCenters, arma::vec initQueryPoint, ControlQueue* simulationQueue, ControlQueue* executionQueue, std::string dictionaryPath, int degOfFreedom, std::vector<double> tmpmys,
						std::vector<double> tmpsigmas, double az, double bz, double stepSize,
						double tolAbsErr, double tolRelErr, double ax, double tau, double ac, arma::vec trajMetricWeights,
                         double maxRelativeToMeanDistance, double as, double alpham) {

    vector<DMPBase> baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);
    dictTraj = new LinCombDmp(initQueryPoint.n_elem, degOfFreedom, dictionaryPath, baseDef, az, bz, trajMetricWeights, timeCenters);

    this->simulationQueue = simulationQueue;
    this->executionQueue = executionQueue;
	this->stepSize = stepSize;
	this->tolAbsErr = tolAbsErr;
	this->tolRelErr = tolRelErr;
	
	this->as = 0.0;
	this->switchTime = 0.0;
	this->newQpSwitch = 1;
	
	this->currentQuery = initQueryPoint;
	this->tEnd = dictTraj->getTmax();
	this->ac = ac;
    this->as = as;
    this->alpham = alpham;
	
	this->maxRelativeToMeanDistance = maxRelativeToMeanDistance;

}

void DictionaryGeneralizer::setAs(double as) {
    this->as = as;
}

DictionaryGeneralizer::DictionaryGeneralizer(arma::vec timeCenters, arma::vec initQueryPoint, ControlQueue* simulationQueue, ControlQueue* executionQueue, std::string dictionaryPath, int degOfFreedom, std::vector<double> tmpmys, std::vector<double> tmpsigmas, double az, double bz,
                  double stepSize, double tolAbsErr, double tolRelErr, double ax, double tau, double ac, double as, arma::mat metric, double maxRelativeToMeanDistance, double alpham) {
	
    vector<DMPBase> baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);
    dictTraj = new LinCombDmp(initQueryPoint.n_elem, degOfFreedom, dictionaryPath, baseDef, az, bz, metric, timeCenters);

    this->simulationQueue = simulationQueue;
    this->executionQueue = executionQueue;
	this->stepSize = stepSize;
	this->tolAbsErr = tolAbsErr;
	this->tolRelErr = tolRelErr;
	
	this->as = 0.0;
	this->switchTime = 0.0;
	this->newQpSwitch = 1;
	
	this->currentQuery = initQueryPoint;
	this->tEnd = dictTraj->getTmax();
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

t_executor_res DictionaryGeneralizer::simulateTrajectory() {
    return executeGen(currentQuery, tEnd, ac, as, 1);
}

// TODO: implement execute trajectory
t_executor_res DictionaryGeneralizer::executeTrajectory() {
    return executeGen(currentQuery, tEnd, ac, as, 0);
}

void DictionaryGeneralizer::setTrajectory(Trajectory* traj) {
	
	// TODO: check problem that occurred here with just casting and assigning pointer
    dictTraj = (LinCombDmp*) dictTraj->copy();
	dictTraj->setMetric((dynamic_cast<LinCombDmp*>(traj))->getMetric());
	
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
    distanceCoeffs = distanceCoeffs / distanceCoeffs.max();

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

    return distanceCoeffs;

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
t_executor_res DictionaryGeneralizer::executeGen(arma::vec query, double tEnd, double ac, double as, int simulate) {
	
    int firstTime = 1;
	double norm = 0.0;
	currentTime = 0.0;
	int points = getQueryPointCount();
    int degOfFreedom = dictTraj->getDegreesOfFreedom();

    vec timeCenters = dictTraj->getTimeCenters();
    int centersCount = timeCenters.n_elem;
    int querySize = query.n_elem;

	vector<DMPExecutor*> execs;
	currentQuery = query;
	newQpSwitch = 1;

	oldCoefficients = vec(points);
	newCoefficients = vec(points);
	currentCoefficients = vec(points);
	
    int isFirstIteration = 1;

//	double alpham = 1.0;

//    cout << "(DictionaryGeneralizer) starting generalized execution" << endl;

	t_executor_res ret;
	//vector<vector<double>> retY;
	vector<double>* retY = new vector<double>[degOfFreedom];
	vector<double> retT;

	// create all executors
	for(int i = 0; i < points; ++i) {

        QueryPoint currentQp = dictTraj->getQueryPoints().at(i);
        DMPExecutor* currentExec = new DMPExecutor(currentQp.getDmp());
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

    //extendedMetricMat(0,0) = 1.0; extendedMetricMat(0,1) = 1.0; extendedMetricMat(1,0) = 1.0; extendedMetricMat(1,1) = 1.0;

    // for segmentation test
//    extendedMetricMat(0,0) = 1.0; extendedMetricMat(0,1) = 0.9589; extendedMetricMat(1,0) = 0.9589; extendedMetricMat(1,1) = 0.9449;
//    extendedMetricMat(2,2) = 1.0; extendedMetricMat(2,3) = 0.7364; extendedMetricMat(3,2) = 0.7364; extendedMetricMat(3,3) = 4.1484;


    // erste matrix (beginn), zweite matrix (ende)
    /*
    extendedMetricMat(0,0) = 1.0; extendedMetricMat(0,1) = 1.0; extendedMetricMat(1,0) = 1.0; extendedMetricMat(1,1) = 1.0;
    extendedMetricMat(2,2) = 1.0; extendedMetricMat(2,3) = 1.0; extendedMetricMat(3,2) = 1.0; extendedMetricMat(3,3) = 1.0;
    extendedMetricMat(4,4) = 1.0; extendedMetricMat(4,5) = 1.4333; extendedMetricMat(5,4) = 1.4333; extendedMetricMat(5, 5) = 2.0938;
    */

    int correspondingIdx = -1;
    int oldCorrespondingIdx = -1;
    extendedQuery.fill(0.0);
    Mahalanobis extendedMetric(extendedMetricMat);

	// execute dmps and compute linear combination
    for(; currentTime < tEnd && !stopExecution; currentTime += stepSize) {

        oldCorrespondingIdx = correspondingIdx;
        correspondingIdx = computeClosestT(currentTime, timeCenters);

        if(oldCorrespondingIdx != correspondingIdx) {

            extendedQuery = computeExtendedQuery(currentTime, correspondingIdx, currentQuery);
            firstTime = true;

        }

		vec distanceCoeffs(points);
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

        /**************actual computation of trajectory using coefficients***********/
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

        ControlQueue* queue = NULL;
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
			retY[i].push_back(nextJoints(i));

		retT.push_back(currentTime);
		
	}

	for(int i = 0; i < degOfFreedom; ++i)
		ret.y.push_back(stdToArmadilloVec(retY[i]));
	
	ret.t = stdToArmadilloVec(retT);
	
	// clean up
	for(int i = 0; i < points; ++i) {
		execs.at(i)->destroyIntegration();
		delete execs.at(i);
	}

	currentTime = 0.0;

	return ret;

}

Trajectory* DictionaryGeneralizer::getTrajectory() {
	return dictTraj;
}
