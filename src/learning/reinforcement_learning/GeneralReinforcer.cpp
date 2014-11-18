#include "GeneralReinforcer.h"

#include "DMPReinforcer.h"
#include "../../utils/gnuplot-cpp/gnuplot_i.hpp"

using namespace std;
using namespace arma;

GeneralReinforcer::GeneralReinforcer(std::shared_ptr<TrajectoryExecutor> trajEx, std::shared_ptr<CostComputer> cost, std::shared_ptr<ControlQueue> simulationQueue, std::shared_ptr<ControlQueue> executionQueue) {
	
	this->trajEx = trajEx;
	this->cost = cost;
    this->simulationQueue = simulationQueue;
    this->executionQueue = executionQueue;
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

std::vector<std::shared_ptr<Trajectory>> GeneralReinforcer::getLastRolloutParameters() {
	return rollout;
}

std::vector<t_executor_res> GeneralReinforcer::getLastExecutionResults() {
	return dmpResult;
}

t_executor_res GeneralReinforcer::getLastUpdateRes() {
	return lastUpdateRes;
}

void GeneralReinforcer::performRollout(int doSimulation, int doExecution) {
	
    char cont = 'n';
	
	if(isFirstIteration) {
		
		rollout = getInitialRollout();
		
	}
	else {
		
		rollout = computeRolloutParamters();

	}

	lastCost.clear();
	dmpResult.clear();

    int degFreedom = rollout.at(0)->getDegreesOfFreedom();
    arma::vec startingJoints = arma::vec(degFreedom);

    for(int k = 0; k < rollout.size(); ++k) {

        vec startingPos = rollout.at(k)->getStartingPos();
        startingJoints = startingPos;

//        cout << "(DMPReinforcer) performing rollout " << k << endl;

        t_executor_res simRes;
        if(doSimulation) {
        //    cout << "(DMPReinforcer) simulating rollout" << endl;
            simulationQueue->moveJoints(startingJoints);
            trajEx->setTrajectory(rollout.at(k));
            simRes = trajEx->simulateTrajectory();

            if(!doExecution) {
            //    dmpResult.push_back(simRes);
                if(isFirstIteration)
                    lastUpdateRes = simRes;
            }

		}

        bool useRollout = true;
        if(doExecution) {
			
			cout << "(DMPReinforcer) do you want to execute this trajectory? (y/N) ";
			cin >> cont;
		
            if(cont == 'y' || cont == 'Y') {
				
                cout << "(DMPReinforcer) executing rollout" << endl;
				
                simulationQueue->moveJoints(startingJoints);
				
				trajEx->setTrajectory(rollout.at(k));
                simRes = trajEx->executeTrajectory();
                useRollout = true;
				
            } else {
                useRollout = false;
            }
			
		}

        if(doSimulation || doExecution) {
            dmpResult.push_back(simRes);
            t_executor_res resK = simRes;
            double delta = cost->computeCost(resK);
            lastCost.push_back(delta);

            if(isFirstIteration) {
                lastUpdateRes = resK;
                lastUpdateCost = delta;
            }

        }
        /*
        else {
            cout << "(GeneralReinforcer) rollout number " << k << " rollout not used" << endl;
        //    --k;
        }
        */

        if(doSimulation)
            simulationQueue->moveJoints(startingJoints);
        if(doExecution)
            executionQueue->moveJoints(startingJoints);

        if(doExecution) {
            cout << "(GeneralReinforcer) press a key to perform next rollout (rollout number " << (k + 2) << ")" << endl;
            getchar();
            getchar();
        }

	}

	double tmpCost = lastUpdateCost;
    std::shared_ptr<Trajectory> tmpUpdate = lastUpdate->copy();
    t_executor_res tmpRes = lastUpdateRes;
    lastUpdate = updateStep();

    trajEx->setTrajectory(lastUpdate);

    if(!isFirstIteration) {
        cout << "(GeneralReinforcer) performing newest update" << endl;

        t_executor_res simRes;

        if(doSimulation) {
            cout << "(DMPReinforcer) simulating update" << endl;
            simulationQueue->moveJoints(startingJoints);
            simRes = trajEx->simulateTrajectory();

            if(!doExecution) {
                lastUpdateRes = simRes;
            }
        }

        if(doExecution) {

            cout << "(DMPReinforcer) do you want to execute this update? (y/N) ";
            cin >> cont;

            if(cont == 'y' || cont == 'Y') {

                cout << "(DMPReinforcer) executing update" << endl;

                simulationQueue->moveJoints(startingJoints);
                simRes = trajEx->executeTrajectory();

            } else {
            //    throw "(GeneralReinforcer) update not usable (stopping reinforcement learning)";
            }

        }

        if(doSimulation || ( doExecution && (cont == 'y' || cont == 'Y') )) {
            lastUpdateRes = simRes;
            lastUpdateCost = cost->computeCost(lastUpdateRes);
        }

    }


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


    isFirstIteration = false;
	
    cout << "(DMPReinforcer) last update reward/cost: " << lastUpdateCost << endl;

}

double GeneralReinforcer::getLastUpdateReward() {
	return lastUpdateCost;
}

std::shared_ptr<Trajectory> GeneralReinforcer::getLastUpdate() {
	
	return lastUpdate;
	
}

void GeneralReinforcer::setLastUpdate(std::shared_ptr<Trajectory> lastUpdate) {
	this->lastUpdate = lastUpdate;
}
