#ifndef QUERYPOINT
#define QUERYPOINT

#include "DMP.h"
#include <vector>
#include <armadillo>
#include <string>

/*
struct t_querypoint {
	
	std::string fileQueryPath;
	std::string fileDataPath;
	
	double ac;
	double ax;
	double az;
	double bz;
	
	t_learned_dmp dmp;
	arma::vec queryPoint;
	
};
*/

class QueryPoint {
	
private:
	
	std::string fileQueryPath;
	std::string fileDataPath;
    std::string fileDmpPath;
	
    std::shared_ptr<Dmp> internalDmp;
	arma::vec queryPoint;
	
public:
	
    QueryPoint(std::string fileQueryPath, std::string fileDataPath, std::string fileDmpPath, std::shared_ptr<Dmp> dmp, arma::vec queryPoint);
	QueryPoint(const QueryPoint& qp);
	
	std::string getFileQueryPath();
	std::string getFileDataPath();
    std::string getFileDmpPath();
	
	void setQueryPoint(arma::vec queryPoint);
	arma::vec getQueryPoint();
	
    void setDmp(std::shared_ptr<Dmp> dmp);
    std::shared_ptr<Dmp> getDmp();
	
	
};

#endif
