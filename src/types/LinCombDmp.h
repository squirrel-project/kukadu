#ifndef LINCOMPDMP
#define LINCOMPDMP

#include "DictionaryTrajectory.h"
#include "../trajectory/DMPTrajectoryComparator.h"
#include "../learning/metric_learning/Mahalanobis.h"
#include "../learning/metric_learning/TogersonMetricLearner.h"
#include "../utils/types.h"
#include "../utils/utils.h"
#include "../utils/conversion_utils.h"
#include <vector>
#include <armadillo>

class LinCombDmp : public DictionaryTrajectory {
	
private:
	
    std::vector<Mahalanobis> metric;
	arma::vec currentQueryPoint;
	arma::vec trajMetricWeights;
    arma::vec timeCenters;
	
public:
	
    LinCombDmp(int queryDegOfFreedom, int degOfFreedom, std::string baseFolder, std::vector<DMPBase> baseDef, double az, double bz,
        arma::vec trajMetricWeights, arma::vec timeCenters
	);
	
    LinCombDmp(int queryDegOfFreedom, int degOfFreedom, std::string baseFolder, std::vector<DMPBase> baseDef, double az, double bz,
        arma::mat metricM, arma::vec timeCenters
	);
	
	LinCombDmp(const LinCombDmp& copy);
	LinCombDmp();
	
	void setCurrentQueryPoint(arma::vec currQuery);
	arma::vec getCurrentQueryPoint();
    arma::vec getTimeCenters();
	
	
	std::vector<arma::vec> getCoefficients();
	void setCoefficients(std::vector<arma::vec> coeffs);
	void initializeMetric();
	
	int getQueryDegreesOfFreedom() const;
	
    std::vector<Mahalanobis> getMetric();
    void setMetric(Mahalanobis metric);
    void setMetric(std::vector<Mahalanobis> metric);
	
	int operator==(LinCombDmp const& comp) const;
	
	Trajectory* copy();
	
};

#endif
