#include "conversion_utils.h"

using namespace std;
using namespace arma;

int compareVectorOfArmadillos(std::vector<arma::vec> vec1, std::vector<arma::vec> vec2) {
	if(vec1.size() != vec2.size()) return 0;
	for(int i = 0; i < vec1.size(); ++i)
		if(!compareArmadilloVec(vec1.at(i), vec2.at(i))) return 0;
	return 1;
}

int compareArmadilloVec(arma::vec vec1, arma::vec vec2) {
	if(vec1.n_elem != vec2.n_elem) return 0;
	for(int i = 0; i < vec1.n_elem; ++i)
		if(vec1(i) != vec2(i)) return 0;
	return 1;
}

int compareVectorOfDoubles(std::vector<double> vec1, std::vector<double> vec2) {
	if(vec1.size() != vec2.size()) return 0;
	for(int i = 0; i < vec1.size(); ++i)
		if(vec1.at(i) != vec2.at(i)) return 0;
	return 1;
}