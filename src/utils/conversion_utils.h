#ifndef CONVERSIONUTILS
#define CONVERSIONUTILS

#include <vector>
#include <armadillo>

int compareArmadilloVec(arma::vec vec1, arma::vec vec2);
int compareVectorOfArmadillos(std::vector<arma::vec> vec1, std::vector<arma::vec> vec2);
int compareVectorOfDoubles(std::vector<double> vec1, std::vector<double> vec2);

#endif