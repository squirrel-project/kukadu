#include "DictionaryGeneralizer.h"

using namespace std;
using namespace arma;

DictionaryGeneralizer::DictionaryGeneralizer(arma::vec initQueryPoint, ControlQueue* queue, std::string dictionaryPath, int degOfFreedom, std::vector<double> tmpmys,
						std::vector<double> tmpsigmas, double az, double bz, double stepSize,
						double tolAbsErr, double tolRelErr, double ax, double tau, double ac, arma::vec trajMetricWeights,
					     double maxRelativeToMeanDistance, double as) {

	vector<DMPBase> baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);
	dictTraj = new LinCombDmp(initQueryPoint.n_elem, degOfFreedom, dictionaryPath, baseDef, az, bz, ax, tau, trajMetricWeights);

	this->queue = queue;
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
	
	this->maxRelativeToMeanDistance = maxRelativeToMeanDistance;

}

DictionaryGeneralizer::DictionaryGeneralizer(arma::vec initQueryPoint, ControlQueue* queue, std::string dictionaryPath, int degOfFreedom, std::vector<double> tmpmys, std::vector<double> tmpsigmas, double az, double bz,
			      double stepSize, double tolAbsErr, double tolRelErr, double ax, double tau, double ac, double as, arma::mat metric, double maxRelativeToMeanDistance) {
	
	vector<DMPBase> baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);
	dictTraj = new LinCombDmp(initQueryPoint.n_elem, degOfFreedom, dictionaryPath, baseDef, az, bz, ax, tau, metric);

	this->queue = queue;
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
	
	this->maxRelativeToMeanDistance = maxRelativeToMeanDistance;
	
}

void DictionaryGeneralizer::switchQueryPoint(vec query) {
	
	switcherMutex.lock();
		currentQuery = query;
		switchTime = 0.0;
		newQpSwitch = 1;
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
	return executeGen(currentQuery, tEnd, ac, as);
}

// TODO: implement execute trajectory
t_executor_res DictionaryGeneralizer::executeTrajectory() {
}

void DictionaryGeneralizer::setTrajectory(Trajectory* traj) {
	
	// TODO: check problem that occurred here with just casting and assigning pointer
	dictTraj->setMetric((dynamic_cast<LinCombDmp*>(traj))->getMetric());
	
}

double DictionaryGeneralizer::getCurrentTime() {
	return currentTime;
}

// TODO: find out why metric can produce negative distances after reinforcement learning
t_executor_res DictionaryGeneralizer::executeGen(arma::vec query, double tEnd, double ac, double as) {
	
    int firstTime = 1;
	double norm = 0.0;
	currentTime = 0.0;
	int points = getQueryPointCount();
	int degOfFreedom = dictTraj->getDegreesOfFreedom();

	vector<DMPExecutor*> execs;
	currentQuery = query;
	newQpSwitch = 1;
	
	oldCoefficients = vec(points);
	newCoefficients = vec(points);
	currentCoefficients = vec(points);
	
	// simulation is standard option
	int simulate = 1;
    int isFirstIteration = 1;
	
	double alpham = 1.0;
	
	// if queue is defined, execute trajectory
	if(this->queue) simulate = 0;
	
    cout << "(DictionaryGeneralizer) starting generalized execution" << endl;

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
	
	mat Z = columnToSquareMatrix(dictTraj->getCoefficients().at(0));
	Mahalanobis metric(Z.t() * Z);
	
//	cout << "current query: " << currentQuery << endl;

	// execute dmps and compute linear combination
	for(; currentTime < tEnd; currentTime += stepSize) {

        if((currentTime - ((int) currentTime)) < 0.02)
            cout << "(DictionaryGeneralizer) execution of trajectory (currentTime: " << currentTime << ")" << endl;
		
		vec distanceCoeffs(points);
		vec nextJoints(degOfFreedom);
		nextJoints.fill(0.0);
		
		// perform coefficient switching (mutex for the sake of thread safety)
		switcherMutex.lock();
		
			// compute all distances
			for(int i = 0; i < points; ++i) {
                double currCoeff = metric.computeSquaredDistance(dictTraj->getQueryPoints().at(i).getQueryPoint(), currentQuery);
                distanceCoeffs(i) = currCoeff;
			}
//			cout << distanceCoeffs.t() << endl;
			
			// compute average distance
			double avgDist = 0.0;
			for(int i = 0; i < points; ++i)
				avgDist += distanceCoeffs(i);
			
			avgDist = avgDist / points;
			
			// if loop is executed first time, initialize everything
			if(firstTime) {
				
				dictTraj->setCurrentQueryPoint(currentQuery);
				
                for(int i = 0; i < points; ++i) {
                    double currCoeff = 1 / exp(alpham * distanceCoeffs(i));
                    currentCoefficients(i) = currCoeff;
                }
				
				// drop trajectories that are too far away
                double tolerableDistance = avgDist * maxRelativeToMeanDistance;
                for(int i = 0; i < points; ++i) {
                    double currCoeff = distanceCoeffs(i);
                    if(currCoeff > tolerableDistance) {
						currentCoefficients(i) = 0.0;
                    }
                }

				oldCoefficients = newCoefficients = currentCoefficients;
				
				firstTime = 0;
				
			}
			
			// if new query point has been set
			if(newQpSwitch) {
				
				newQpSwitch = 0;
				
				for(int i = 0; i < points; ++i)
					newCoefficients(i) = 1 / exp(alpham * distanceCoeffs(i));
				
				// drop trajectories that are too far away
				for(int i = 0; i < points; ++i)
					if(distanceCoeffs(i) > (avgDist * maxRelativeToMeanDistance))
						newCoefficients(i) = 0.0;
				
				oldCoefficients = currentCoefficients;
				
			}

            if(dot(currentCoefficients, currentCoefficients) == 0.0) {
                string errStr = "(DictionaryGeneralizer) all coefficients are 0, please recheck your settings";
                cerr << errStr << endl;
                throw "(DictionaryGeneralizer) all coefficients are 0, please recheck your settings";
            }

			// compute coefficient switching
			currentCoefficients = (1 - exp(- as * switchTime)) * newCoefficients + oldCoefficients * exp(- as * switchTime);
			
			// compute new normalization
			norm = 0.0;
			for(int i = 0; i < points; ++i) {
                double currCoeff = currentCoefficients(i);
                norm += currCoeff;
			}
			
			switchTime += stepSize;
			
		switcherMutex.unlock();

		for(int i = 0; i < points; ++i) {
			
			vec currJoints(degOfFreedom);
			currJoints.fill(0.0);
			currJoints = execs.at(i)->doIntegrationStep(ac);

            double currCoeff = currentCoefficients(i);
            nextJoints += currCoeff / norm * currJoints;

            // query points not correct
/*
            if(currCoeff/norm > 0.5)
                cout << "influental query point: " << dictTraj->getQueryPoints().at(i).getQueryPoint().t() << endl;
            else
                cout << "non-influental query point: " << dictTraj->getQueryPoints().at(i).getQueryPoint().t() << endl;
*/
        }

        if(!simulate) {

            // if real robot execution and first integration step --> move to initial position
            if(isFirstIteration) {

                cout << "(DictionaryGeneralizer) moving to initial execution position" << endl;
                float* startingJoints = new float[nextJoints.n_elem];
                for(int i = 0; i < nextJoints.n_elem; ++i) startingJoints[i] = nextJoints(i);
                queue->moveJoints(startingJoints);
                isFirstIteration = 0;
                cout << "(DictionaryGeneralizer) starting trajectory execution" << endl;

            } else {

                // if not first integration step but real robot execution --> add new positions to queue
                float* moveJoints = new float[nextJoints.n_elem];

                // move to desired position
                for(int i = 0; i < nextJoints.n_elem; ++i) {
                    moveJoints[i] = nextJoints(i);
                }

                // synchronize to control queue (maximum one joint array has to be already in there --> needed for phase stopping such that DMPExecutor does not progress to fast)
                queue->synchronizeToControlQueue(0);
                queue->addJointsPosToQueue(moveJoints);

                float* currentJoints = queue->getCurrentJoints().joints;
                for(int i = 0; i < nextJoints.n_elem; ++i) {
                    nextJoints(i) = currentJoints[i];
                }

            }
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
