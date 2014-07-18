#ifndef CONVERSIONUTILS
#define CONVERSIONUTILS

#include <vector>
#include <armadillo>
#include <iostream>
#include <fstream>
#include "../utils/Tokenizer.h"

int compareArmadilloVec(arma::vec vec1, arma::vec vec2);
int compareVectorOfArmadillos(std::vector<arma::vec> vec1, std::vector<arma::vec> vec2);
int compareVectorOfDoubles(std::vector<double> vec1, std::vector<double> vec2);
double string_to_double(const std::string& s);
std::string double_to_string(const double d);
arma::mat readMat(std::ifstream& inStream);

#endif
