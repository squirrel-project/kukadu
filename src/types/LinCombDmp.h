#ifndef KUKADU_LINCOMPDMP_H
#define KUKADU_LINCOMPDMP_H

#include <vector>
#include <armadillo>

#include "../utils/types.h"
#include "../utils/utils.h"
#include "../utils/conversion_utils.h"
#include "../types/DictionaryTrajectory.h"
#include "../trajectory/DMPTrajectoryComparator.h"
#include "../learning/metric_learning/Mahalanobis.h"
#include "../learning/metric_learning/TogersonMetricLearner.h"

class LinCombDmp : public DictionaryTrajectory {
	
private:

    arma::vec timeCenters;
	arma::vec currentQueryPoint;
	arma::vec trajMetricWeights;

    std::vector<Mahalanobis> metric;

public:
	
    LinCombDmp();
    LinCombDmp(const LinCombDmp& copy);
    LinCombDmp(std::string baseFolder, double az, double bz, arma::mat metricM, arma::vec timeCenters);
    LinCombDmp(int queryDegOfFreedom, std::string baseFolder, double az, double bz, arma::vec trajMetricWeights, arma::vec timeCenters);
	
    void initializeMetric();
    void setMetric(Mahalanobis metric);
    void setCurrentQueryPoint(arma::vec currQuery);
    void setMetric(std::vector<Mahalanobis> metric);
    void setCoefficients(std::vector<arma::vec> coeffs);

    int getQueryDegreesOfFreedom() const;
    int operator==(LinCombDmp const& comp) const;

    arma::vec getTimeCenters();
	arma::vec getCurrentQueryPoint();

    std::vector<Mahalanobis> getMetric();
	std::vector<arma::vec> getCoefficients();

    KUKADU_SHARED_PTR<Trajectory> copy();
	
};

#endif
