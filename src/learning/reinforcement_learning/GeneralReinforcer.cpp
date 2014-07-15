#include "GeneralReinforcer.h"

#include "DMPReinforcer.h"
#include "../../utils/gnuplot-cpp/gnuplot_i.hpp"

using namespace std;
using namespace arma;

GeneralReinforcer::GeneralReinforcer(TrajectoryExecutor* trajEx, CostComputer* cost, ControlQueue* movementQueue) {
	
	this->trajEx = trajEx;
	this->cost = cost;
	this->movementQueue = movementQueue;
	this->isFirstIteration = true;
	this->lastCost.push_back(-1.0);
	this->lastUpdateCost = 0.0;
	
}

bool GeneralReinforcer::getIsFirstIteration() {
	return isFirstIteration;
}

std::vector<double> GeneralReinforcer::getLastRolloutCost() {
	return lastCost;
}

std::vector<Trajectory*> GeneralReinforcer::getLastRolloutParameters() {
	return rollout;
}

std::vector<t_executor_res> GeneralReinforcer::getLastExecutionResults() {
	return dmpResult;
}

t_executor_res GeneralReinforcer::getLastUpdateRes() {
	return lastUpdateRes;
}

void GeneralReinforcer::performRollout(int doSimulation, int doExecution) {
	
	char cont = 'y';
	vector<Gnuplot*> gs;
	Gnuplot* g1 = NULL;
	
	if(isFirstIteration) {
		
		rollout = getInitialRollout();
		
    //	trajEx->setTrajectory(rollout.at(0));
    //	lastUpdateRes = trajEx->simulateTrajectory();
		
		
	}
	else {
		
		rollout = computeRolloutParamters();
		
	}
	
	lastCost.clear();
	dmpResult.clear();
	
	for(int k = 0; k < rollout.size(); ++k) {

//        cout << "(DMPReinforcer) performing rollout " << k << endl;

		if(doSimulation) {

			trajEx->setTrajectory(rollout.at(k));
			t_executor_res simRes = trajEx->simulateTrajectory();
			dmpResult.push_back(simRes);

            if(isFirstIteration)
                lastUpdateRes = simRes;

		}

        if(doExecution) {

            dmpResult.clear();
			
			cout << "(DMPReinforcer) do you want to execute this trajectory? (y/N) ";
			cin >> cont;
		
			if(doExecution && (cont == 'y' || cont == 'Y')) {
				
				cout << "(DMPReinforcer) executing rollout" << endl;
				
				int degFreedom = rollout.at(k)->getDegreesOfFreedom();
				vec startingPos = rollout.at(k)->getStartingPos();
				double* tmp = new double[degFreedom];
				for(int i = 0; i < rollout.at(k)->getDegreesOfFreedom(); ++i)
					tmp[i] = startingPos(i);
				
				float* startingJoints = new float[degFreedom];
				for(int i = 0; i < degFreedom; ++i) startingJoints[i] = tmp[i];
				
				movementQueue->setStartingJoints(startingJoints);
				movementQueue->setStiffness(2200, 300, 1.0, 15000, 150, 2.0);
				thread* thr = movementQueue->startQueueThread();
				
				trajEx->setTrajectory(rollout.at(k));
				t_executor_res simRes = trajEx->executeTrajectory();
				dmpResult.push_back(simRes);
				
				movementQueue->setFinish();
				thr->join();
				
			}
			
		}
		
        t_executor_res resK = dmpResult.at(k);
        double delta = cost->computeCost(resK);
		lastCost.push_back(delta);

	}
	
	double tmpCost = lastUpdateCost;
	Trajectory* tmpUpdate = lastUpdate;
	t_executor_res tmpRes = lastUpdateRes;
	
	lastUpdate = updateStep();
	
	trajEx->setTrajectory(lastUpdate);

    if(!isFirstIteration) {
        cout << "(GeneralReinforcer) performing newest update" << endl;
        lastUpdateRes = trajEx->simulateTrajectory();
    }
	
	lastUpdateCost = cost->computeCost(lastUpdateRes);

    // TODO: this is a hack!!!! repair it (power cannot directly be applied to metric learning) --> results can get worse instead of better
    if(lastUpdateCost < tmpCost) {

		lastUpdateCost = tmpCost;
		lastUpdate = tmpUpdate;
		lastUpdateRes = tmpRes;
		
		// get best reward
		for(int i = 0; i < lastCost.size(); ++i) {
			if(lastCost.at(i) > tmpCost) {
				lastUpdateCost = lastCost.at(i);
				lastUpdate = rollout.at(i);
				lastUpdateRes = dmpResult.at(i);
			}
		}

    }

	this->lastUpdate = lastUpdate;
    isFirstIteration = false;
	
    cout << "(DMPReinforcer) last update reward/cost: " << lastUpdateCost << endl;

}

double GeneralReinforcer::getLastUpdateReward() {
	return lastUpdateCost;
}

Trajectory* GeneralReinforcer::getLastUpdate() {
	
	return lastUpdate;
	
}

void GeneralReinforcer::setLastUpdate(Trajectory* lastUpdate) {
	this->lastUpdate = lastUpdate;
}
