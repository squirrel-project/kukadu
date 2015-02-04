#include "GenDMPReinforcer.h"
#include "../../utils/gnuplot-cpp/gnuplot_i.hpp"

#define DEBUGGENDMPREINFORCER 1

using namespace std;
using namespace arma;

GenDMPReinforcer::GenDMPReinforcer(vec initialQueryPoint, CostComputer* cost, DMPGeneralizer* dmpGen, GenericKernel* trajectoryKernel, GenericKernel* parameterKernel, std::shared_ptr<ControlQueue> movementQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) : DMPReinforcer(cost, movementQueue, ac, dmpStepSize, tolAbsErr, tolRelErr) {
	this->dmpGen = dmpGen;
	this->trajectoryKernel = trajectoryKernel;
	this->parameterKernel = parameterKernel;
	this->initialQueryPoint = initialQueryPoint;
	this->lastQueryPoint = initialQueryPoint;
	
	this->ql = getQMin();
	this->qh = getQMax();
	this->genResults = NULL;
	
	this->isFirstRolloutAfterInit = true;
    this->simQueue = std::shared_ptr<PlottingControlQueue>(new PlottingControlQueue(dmpGen->getDegOfFreedom(), dmpStepSize));

}

std::vector<Dmp> GenDMPReinforcer::getInitialRollout() {
	
	cout << "(GenDMPReinforcer) initial query point: " << initialQueryPoint(0) << endl;
	
	genResults = new t_executor_res[dmpGen->getQueryPointCount()];
	for(int i = 0; i < dmpGen->getQueryPointCount(); ++i) {
		Dmp queryDmp = dmpGen->getQueryPointByIndex(i).getDmp();
        DMPExecutor dmpsim(queryDmp, simQueue);
		genResults[i] = dmpsim.simulateTrajectory(0, queryDmp.getTmax(), getDmpStepSize(), getTolAbsErr(), getTolRelErr());
	}
	
	vector<Dmp> ret;
	Dmp rollout = dmpGen->generalizeDmp(trajectoryKernel, parameterKernel, initialQueryPoint, 100000);
    DMPExecutor dmpsim(rollout, simQueue);
	t_executor_res dmpResult = dmpsim.simulateTrajectory(0, rollout.getTmax(), getDmpStepSize(), getTolAbsErr(), getTolRelErr());
	
	if(DEBUGGENDMPREINFORCER) {
		
		plotFeedback(dmpGen, rollout, dmpResult);
		
	}
	
	lastUpdate = rollout;
	ret.push_back(rollout);
	return ret;
	
}

double GenDMPReinforcer::getQMin() {
	
	double qMin = DBL_MAX;
	
	int pointCount = dmpGen->getQueryPointCount();
	for(int i = 0; i < pointCount; ++i) {
		double currentQp = dmpGen->getQueryPointByIndex(i).getQueryPoint()(0);
		if(currentQp < qMin) qMin = currentQp;
	}
	
	return qMin;
	
}

double GenDMPReinforcer::getQMax() {
	
	double qMax = 0.0;
	
	int pointCount = dmpGen->getQueryPointCount();
	for(int i = 0; i < pointCount; ++i) {
		double currentQp = dmpGen->getQueryPointByIndex(i).getQueryPoint()(0);
		if(currentQp > qMax) qMax = currentQp;
	}
	
	return qMax;
	
}

Dmp GenDMPReinforcer::updateStep() {
	
	double lastCost = getLastRolloutCost().at(0);
	double q;
	
	if(isFirstRolloutAfterInit) {
		char cont = 'N';
		cout << "do you want add stronger restriction on ql or qh? (y/N)";
		cin >> cont;
		
		if(cont == 'y' || cont == 'Y') {
			cout << "enter ql value...";
			cin >> ql;
			cout << "enter qh value...";
			cin >> qh;
		}
		isFirstRolloutAfterInit = false;
	}
	
	if(lastCost > 0) {
		
		ql = lastQueryPoint(0);
		q = ql + (qh - ql) * 0.618;
		
	} else {
		
		qh = lastQueryPoint(0);
		q = ql + (qh - ql) * 0.382;
		
	}
	
	lastQueryPoint(0) = q;
	
	cout << "(GenDMPReinforcer) next rollout query point: " << q << endl;
	
	vector<Dmp> ret;
	Dmp rollout = dmpGen->generalizeDmp(trajectoryKernel, parameterKernel, lastQueryPoint, 100000);
	
    DMPExecutor dmpsim(rollout, simQueue);
	t_executor_res dmpResult = dmpsim.simulateTrajectory(0, rollout.getTmax(), getDmpStepSize(), getTolAbsErr(), getTolRelErr());
	
	if(DEBUGGENDMPREINFORCER) {
		
		plotFeedback(dmpGen, rollout, dmpResult);
		
	}
	
	lastUpdate = rollout;
	
	ret.push_back(rollout);
	return rollout;
	
}

Dmp GenDMPReinforcer::getLastUpdate() {
	return getLastRolloutParameters().at(0);
}

std::vector<Dmp> GenDMPReinforcer::computeRolloutParamters() {
	
	vector<Dmp> ret;
	ret.push_back(lastUpdate);
	return ret;
	
}

void GenDMPReinforcer::plotFeedback(DMPGeneralizer* dmpGen, Dmp rollout, t_executor_res currentRolloutRes) {
	
	vector<Gnuplot*> gs;
	Gnuplot* g1;
	
	for(int plotTraj = 0; plotTraj < rollout.getDegreesOfFreedom(); ++plotTraj) {
				
		ostringstream convert;   // stream used for the conversion
		convert << plotTraj;
		
		string title = string("fitted sensor data (joint") + convert.str() + string(")");
		g1 = new Gnuplot(title);
		gs.push_back(g1);
		
		for(int i = 0; i < dmpGen->getQueryPointCount(); ++i) {
			
			std::ostringstream o;
			o << "sample (";
			
			for(int j = 0; j < dmpGen->getQueryPointByIndex(i).getQueryPoint().n_elem; ++j) {
				vec tmpQp = dmpGen->getQueryPointByIndex(i).getQueryPoint();
				o << tmpQp(j) << " ";
			}
			o << ")";
			g1->set_style("lines").plot_xy(armadilloToStdVec(genResults[i].t), armadilloToStdVec(genResults[i].y[plotTraj]), o.str());
			
		}
		
		g1->set_style("lines").plot_xy(armadilloToStdVec(currentRolloutRes.t), armadilloToStdVec(currentRolloutRes.y[plotTraj]), "executed gen y");
		g1->showonscreen();
		
	}

    getchar();
	
	for(int i = 0; i < gs.size(); ++i) {
		g1 = gs.at(i);
		delete g1;
	}
	
}
