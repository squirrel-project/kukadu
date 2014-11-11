#ifndef DMPTYPES
#define DMPTYPES

#include <vector>
#include <armadillo>
#include <gsl/gsl_matrix.h>

struct mes_result {
	double time;
    arma::vec joints;
};

struct t_executor_res {
	
	arma::vec t;
	std::vector<arma::vec> y;
	arma::vec internalClock;

};

struct trajectory_learner_internal {
	arma::mat desMat;
	arma::vec coeff;
	arma::vec fity;
};

#endif
