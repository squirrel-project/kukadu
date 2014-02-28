#include "LinCombDmp.h"

using namespace std;
using namespace arma;

LinCombDmp::LinCombDmp(int queryDegOfFreedom, int degOfFreedom, std::string baseFolder, std::vector<DMPBase> baseDef, double az, double bz, double ax, double tau,
			arma::vec trajMetricWeights
) :
			DictionaryTrajectory(degOfFreedom, baseFolder, baseDef, az, bz, ax, tau), metric(queryDegOfFreedom) {
	
	this->trajMetricWeights = trajMetricWeights;
	currentQueryPoint = getQueryPoints().at(0).getQueryPoint();
	initializeMetric();
	cout << "(LinCombDmp) constructor" << endl;
}

LinCombDmp::LinCombDmp(int queryDegOfFreedom, int degOfFreedom, std::string baseFolder, std::vector<DMPBase> baseDef, double az, double bz, double ax, double tau,
		arma::mat metricM
	) : DictionaryTrajectory(degOfFreedom, baseFolder, baseDef, az, bz, ax, tau), metric(metricM) {
	
	this->trajMetricWeights = trajMetricWeights;
	currentQueryPoint = getQueryPoints().at(0).getQueryPoint();
	cout << "(LinCombDmp) constructor" << endl;
}

/*
Mahalanobis metric;
	arma::vec currentQueryPoint;
	arma::vec trajMetricWeights;
*/
LinCombDmp::LinCombDmp(const LinCombDmp& copy) : metric(copy.metric) {
	this->currentQueryPoint = copy.currentQueryPoint;
	this->trajMetricWeights = copy.trajMetricWeights;
	this->metric = copy.metric;
//	cout << "(LinCombDmp) copy constructor" << endl;
}

LinCombDmp::LinCombDmp() : metric(0) {
//	cout << "(LinCombDmp) dummy constructor" << endl;
}

void LinCombDmp::setMetric(Mahalanobis metric) {
	this->metric = metric;
}

// TODO: usage of trajectory metric
void LinCombDmp::initializeMetric() {
	
	cout << "(LinCombDmp) metric initialization" << endl;
		
	vector<vec> x1s, x2s;
	vector<double> distances;
	
	vector<QueryPoint> qps = getQueryPoints();
	//Dmp traject1, Dmp traject2, arma::vec degOfFreedomWeights, double integrationStep, double tolAbsErr, double tolRelErr, double tTolerance
	Dmp d1 = qps.at(0).getDmp();
	Dmp d2 = qps.at(1).getDmp();
	
	double integrationStep = d1.getStepSize();
	double tolAbsErr = d1.getTolAbsErr();
	double tolRelErr = d1.getTolRelErr();
	double tTol = 0.5;
	
	DMPTrajectoryComparator trajComp(d1, d2, trajMetricWeights, integrationStep, tolAbsErr, tolRelErr, tTol);
	
	cout << "(LinCombDmp) comparing trajectories" << endl;
	for(int i = 0; i < qps.size(); ++i) {
		
		vec x1 = qps.at(i).getQueryPoint();
		
		for(int j = 0; j < qps.size(); ++j) {
				
			vec x2 = qps.at(j).getQueryPoint();
				
			x1s.push_back(x1);
			x2s.push_back(x2);
			
			trajComp.setTrajectories(qps.at(i).getDmp(), qps.at(j).getDmp(), integrationStep, tolAbsErr, tolRelErr, tTol);
			
			double dist = trajComp.computeDistance();
			cout << "(LimCombDmp) comparation of trajectories " << i << " and " << j << " delivered result " << dist << endl;

			distances.push_back(dist);

		}
	}

	cout << "(LinCombDmp) learning metric" << endl;
	TogersonMetricLearner learner(x1s, x2s, distances);
	metric = learner.learnMetric();
	cout << "metric: " << metric.getM() << endl;
	
	cout << "(LinCombDmp) metric initialization done" << endl;
	
}

// returns metric coefficients
std::vector<arma::vec> LinCombDmp::getCoefficients() {
	
	mat Z = metric.getDecomposition();
	
	vector<vec> ret;
	vec zCoeffs = squareMatrixToColumn(Z);
	ret.push_back(zCoeffs);
	
	return ret;
	
	
	/*
	vector<arma::vec> ret;
	vector<QueryPoint> qps = getQueryPoints();
	vec coeffs(qps.size());
	
	for(int i = 0; i < qps.size(); ++i)
		coeffs(i) = 1 / exp(metric.computeSquaredDistance(qps.at(i).getQueryPoint(), getCurrentQueryPoint()));
	
	ret.push_back(coeffs);
	return ret;
	*/
	
}

// sets metric coefficients
void LinCombDmp::setCoefficients(std::vector<arma::vec> coeffs) {
	
	int k = 0;
	
	// rest is ignored for now
	vec coeffs0 = coeffs.at(0);
	mat newM = columnToSquareMatrix(coeffs0);
	newM = newM * newM.t();
	
	newM = 1 / newM(0,0) * newM;
	
	metric.setM(newM);
	
}

// TODO: write comparator
int LinCombDmp::operator==(LinCombDmp const& comp) const {
	return 0;
}

Trajectory* LinCombDmp::copy() {
	return new LinCombDmp(*this);
}

Mahalanobis LinCombDmp::getMetric() {
	return metric;
}

int LinCombDmp::getQueryDegreesOfFreedom() const {
	return metric.getM().n_cols;
}

void LinCombDmp::setCurrentQueryPoint(arma::vec currQuery) {
	
	currentQueryPoint = currQuery;
	
}

arma::vec LinCombDmp::getCurrentQueryPoint() {
	return currentQueryPoint;
}