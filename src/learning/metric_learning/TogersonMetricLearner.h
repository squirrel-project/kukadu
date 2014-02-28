#ifndef TOGERSON
#define TOGERSON

#include "../../utils/CustomSet.h"
#include "Mahalanobis.h"
#include "MahalanobisLearner.h"

#include <unordered_set>
#include <set>
#include <math.h>
#include <vector>
#include <armadillo>

struct armacomp {
	
	int operator() (const arma::vec& vec1, const arma::vec& vec2) const {
		
		if(vec1.n_elem != vec2.n_elem)
			return vec1.n_elem - vec2.n_elem;
		
		// check equality
		for(int i = 0; i < vec1.n_elem; ++i) {
			if(vec1(i) == vec2(i)) {
				
			} else if (vec1(i) > vec2(i)) {
				std::cout << vec1.t() << " " << vec2.t() << "===============" << 1 << "==============" << std::endl;
				return 1;
			} else {
				std::cout << vec1.t() << " " << vec2.t() << "===============" << -1 << "==============" << std::endl;
				return -1;
			}
		}
		
		std::cout << vec1.t() << " " << vec2.t() << "===============" << 0 << "==============" << std::endl;

		return 0;
		
	}
	
};

// method according to http://forrest.psych.unc.edu/teaching/p230/Torgerson.pdf
class TogersonMetricLearner : public MahalanobisLearner {

private:
	
	int dim;
	int sampleCount;
	
	arma::mat D;
	
//	std::set<arma::vec, armacomp> xsSet;
	CustomSet xsSet;
	
	std::vector<arma::vec> expandedX1s;
	std::vector<arma::vec> expandedX2s;
	std::vector<double> expandedDistances;
	
	void expandConstraints();
	int selectI();
	
	std::vector<int> getDRowIdxs(arma::vec x);
	
	void generateD();
	arma::mat generateX();
	arma::mat generateB(int iIdx);
	arma::mat generateY(arma::mat B);
	arma::mat generateZ(arma::mat X, arma::mat Y);
	arma::mat generateA();
	
	int compareArmadilloVec(arma::vec vec1, arma::vec vec2);
	
public:
	
	TogersonMetricLearner(std::vector<arma::vec> x1s, std::vector<arma::vec> x2s, std::vector<double> distances);
	
	Mahalanobis learnMetric();
	
	
	
};

#endif