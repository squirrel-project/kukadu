#include "LinCombDmp.h"

using namespace std;
using namespace arma;

LinCombDmp::LinCombDmp(int queryDegOfFreedom, std::string baseFolder, double az, double bz,
            arma::vec trajMetricWeights, arma::vec timeCenters
) :
            DictionaryTrajectory(baseFolder, az, bz) {

    for(int i = 0; i < timeCenters.n_elem; ++i)
        metric.push_back(Mahalanobis(queryDegOfFreedom));
	
	this->trajMetricWeights = trajMetricWeights;
	currentQueryPoint = getQueryPoints().at(0).getQueryPoint();
    this->timeCenters = timeCenters;
	initializeMetric();
	cout << "(LinCombDmp) constructor" << endl;

}

LinCombDmp::LinCombDmp(std::string baseFolder, double az, double bz,
        arma::mat metricM, arma::vec timeCenters
    ) : DictionaryTrajectory(baseFolder, az, bz) {

    for(int i = 0; i < timeCenters.n_elem; ++i) {
        metric.push_back(metricM);
    }

	this->trajMetricWeights = trajMetricWeights;
    this->timeCenters = timeCenters;
	currentQueryPoint = getQueryPoints().at(0).getQueryPoint();
    cout << "(LinCombDmp) constructor" << endl;

}

/*
Mahalanobis metric;
	arma::vec currentQueryPoint;
	arma::vec trajMetricWeights;
*/
LinCombDmp::LinCombDmp(const LinCombDmp& copy) : metric(copy.metric), DictionaryTrajectory(copy) {
	this->currentQueryPoint = copy.currentQueryPoint;
	this->trajMetricWeights = copy.trajMetricWeights;
	this->metric = copy.metric;
    this->timeCenters = copy.timeCenters;
//	cout << "(LinCombDmp) copy constructor" << endl;
}

LinCombDmp::LinCombDmp() : metric(0) {
//	cout << "(LinCombDmp) dummy constructor" << endl;
}

/*
void LinCombDmp::setMetric(Mahalanobis metric) {

    this->metric.clear();
    for(int i = 0; i < timeCenters.n_elem; ++i)
        this->metric.push_back(metric);

}
*/

void LinCombDmp::setMetric(std::vector<Mahalanobis> metric) {
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
    Mahalanobis newM = learner.learnMetric();

    for(int i = 0; i < timeCenters.n_elem; ++i)
        metric.push_back(newM);

    cout << "metric: " << newM.getM() << endl;
	
	cout << "(LinCombDmp) metric initialization done" << endl;
	
}

// returns metric coefficients
std::vector<arma::vec> LinCombDmp::getCoefficients() {

    int singleCoeffCount = getQueryDegreesOfFreedom() * getQueryDegreesOfFreedom();
    int queryDegOfFreedom = getQueryDegreesOfFreedom();

    vector<vec> ret;
    vec retVec(timeCenters.n_elem * singleCoeffCount);

    for(int i = 0; i < timeCenters.n_elem; ++i) {

        mat Z = metric.at(i).getDecomposition();
        vec zCoeffs = squareMatrixToColumn(Z);

        for(int j = 0; j < zCoeffs.n_elem; ++j)
            retVec(i * singleCoeffCount + j) = zCoeffs(j);

        /*
        vector<vec> ret;
        vec zCoeffs = symmetricMatrixToColumn(metric.getM());
        ret.push_back(zCoeffs);
        */

    }

    ret.push_back(retVec);
	return ret;
	
}

arma::vec LinCombDmp::getTimeCenters() {
    return timeCenters;
}

// sets metric coefficients
void LinCombDmp::setCoefficients(std::vector<arma::vec> coeffs) {

	int k = 0;
    int singleCoeffCount = getQueryDegreesOfFreedom() * getQueryDegreesOfFreedom();

	// rest is ignored for now
    vec allCeffs0 = coeffs.at(0);
    vec coeffs0(singleCoeffCount);

    metric.clear();
    for(int i = 0; i < timeCenters.n_elem; ++i) {

        for(int j = 0; j < singleCoeffCount; ++j)
            coeffs0(j) = allCeffs0(i * singleCoeffCount + j);


        mat newM = columnToSquareMatrix(coeffs0);
        newM = newM * newM.t();


        //mat newM = columnToSymmetricMatrix(coeffs0);

        newM = 1 / newM(0,0) * newM;
        Mahalanobis mahaNewM(newM);

        metric.push_back(mahaNewM);

    }

}

// TODO: write comparator
int LinCombDmp::operator==(LinCombDmp const& comp) const {
	return 0;
}

std::shared_ptr<Trajectory> LinCombDmp::copy() {
    return std::shared_ptr<Trajectory>(new LinCombDmp(*this));
}

std::vector<Mahalanobis> LinCombDmp::getMetric() {
	return metric;
}

int LinCombDmp::getQueryDegreesOfFreedom() const {
    return metric.at(0).getM().n_cols;
}

void LinCombDmp::setCurrentQueryPoint(arma::vec currQuery) {
	
	currentQueryPoint = currQuery;
	
}

arma::vec LinCombDmp::getCurrentQueryPoint() {
	return currentQueryPoint;
}
