#include "DMPReinforcer.h"
#include "../../utils/gnuplot-cpp/gnuplot_i.hpp"

using namespace std;
using namespace arma;

DMPReinforcer::DMPReinforcer(CostComputer* cost, std::shared_ptr<ControlQueue> movementQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) {
	
	this->cost = cost;
	this->movementQueue = movementQueue;
	this->ac = ac;
	this->dmpStepSize = dmpStepSize;
	this->tolAbsErr = tolAbsErr;
	this->tolRelErr = tolRelErr;
	this->isFirstIteration = true;
	this->lastCost.push_back(-1.0);
	
}

bool DMPReinforcer::getIsFirstIteration() {
	return isFirstIteration;
}

std::vector<double> DMPReinforcer::getLastRolloutCost() {
	return lastCost;
}

std::vector<Dmp> DMPReinforcer::getLastRolloutParameters() {
	return rollout;
}

std::vector<t_executor_res> DMPReinforcer::getLastExecutionResults() {
	return dmpResult;
}

double DMPReinforcer::getDmpStepSize() {
	return dmpStepSize;
}

double DMPReinforcer::getTolAbsErr() {
	return tolAbsErr;
}

double DMPReinforcer::getTolRelErr() {
	return tolRelErr;
}

t_executor_res DMPReinforcer::getLastUpdateRes() {
	return lastUpdateRes;
}

void DMPReinforcer::performRollout(int doSimulation, int doExecution) {
	
	char cont = 'y';
	vector<Gnuplot*> gs;
	Gnuplot* g1 = NULL;
	
	if(isFirstIteration) {
		rollout = getInitialRollout();
		isFirstIteration = false;
		
		DMPExecutor dmpsim(rollout.at(0));
		
		// TODO: switch this to new class scheme (not explicetely use DMPExecutor, but trajectory executor)
		lastUpdateRes = dmpsim.simulateTrajectory(0, rollout.at(0).getTmax(), dmpStepSize, tolAbsErr, tolRelErr);
		
		
	}
	else {
		
		rollout = computeRolloutParamters();
		
	}
	
	lastCost.clear();
	dmpResult.clear();
	
//	cout << "(DMPReinforcer) performing next rollout" << endl;
	for(int k = 0; k < rollout.size(); ++k) {
		
		DMPExecutor dmpsim(rollout.at(k));
		
		if(doSimulation) {

			t_executor_res simRes = dmpsim.simulateTrajectory(0, rollout.at(k).getTmax(), dmpStepSize, tolAbsErr, tolRelErr);
			dmpResult.push_back(simRes);

/*
			for(int plotTraj = 0; plotTraj < rollout.at(k).getDegreesOfFreedom(); ++plotTraj) {

				ostringstream convert;   // stream used for the conversion
				convert << plotTraj;
				
				string title = string("fitted sensor data (joint") + convert.str() + string(")");
				g1 = new Gnuplot(title);
				g1->set_style("lines").plot_xy(armadilloToStdVec(dmpResult.at(k).t), armadilloToStdVec(dmpResult.at(k).y[plotTraj]), "dmp y");
				g1->showonscreen();
				
				gs.push_back(g1);

			}

			for(int i = 0; i < gs.size(); ++i) {
				g1 = gs.at(i);
				delete g1;
			}
			gs.clear();
*/
		}
		
		if(doExecution) {
			
			cout << "(DMPReinforcer) do you want to execute this trajectory? (y/N) ";
			cin >> cont;
		
			if(doExecution && (cont == 'y' || cont == 'Y')) {
				
				cout << "(DMPReinforcer) executing rollout" << endl;
				
                arma::vec startingJoints = rollout.at(k).getY0();
				
				movementQueue->setStartingJoints(startingJoints);
				movementQueue->setStiffness(2200, 300, 1.0, 15000, 150, 2.0);
                std::shared_ptr<std::thread> thr = movementQueue->startQueueThread();
				
				DMPExecutor dmpexec(rollout.at(k));
				dmpResult.push_back(dmpsim.executeTrajectory(ac, 0, rollout.at(k).getTmax(), dmpStepSize, tolAbsErr, tolRelErr, movementQueue));
				
				movementQueue->setFinish();
				thr->join();
				
			}
			
		}
		
		double delta = cost->computeCost(dmpResult.at(k));
		lastCost.push_back(delta);
		
	}
	
	lastUpdate = updateStep();
	DMPExecutor dmpsim(lastUpdate);
	lastUpdateRes = dmpsim.simulateTrajectory(0, lastUpdate.getTmax(), dmpStepSize, tolAbsErr, tolRelErr);
	
	double lastUpdateCost = cost->computeCost(lastUpdateRes);
	
	this->lastUpdate = lastUpdate;
	
	cout << "(DMPReinforcer) last update reward/cost: " << lastUpdateCost << endl;
	
	cout << endl;

}

Dmp DMPReinforcer::getLastUpdate() {
	
	return lastUpdate;
	
}

void DMPReinforcer::setLastUpdate(Dmp lastUpdate) {
	this->lastUpdate = lastUpdate;
}
