#include "QueryPoint.h"

using namespace std;

QueryPoint::QueryPoint(std::string fileQueryPath, std::string fileDataPath, std::string fileDmpPath, Dmp dmp, arma::vec queryPoint) : internalDmp(dmp) {
	
	this->fileQueryPath = fileQueryPath;
	this->fileDataPath = fileDataPath;
	this->queryPoint = queryPoint;
    this->fileDmpPath = fileDmpPath;
	
}

QueryPoint::QueryPoint(const QueryPoint& qp) {
	this->fileQueryPath = qp.fileQueryPath;
	this->fileDataPath = qp.fileDataPath;
	this->queryPoint = qp.queryPoint;
	this->internalDmp = qp.internalDmp;
    this->fileDmpPath = qp.fileDmpPath;
}

std::string QueryPoint::getFileQueryPath() {
	return fileQueryPath;
}

std::string QueryPoint::getFileDataPath() {
	return fileDataPath;
}

std::string QueryPoint::getFileDmpPath() {
    return fileDmpPath;
}

arma::vec QueryPoint::getQueryPoint() {
	return queryPoint;
}

Dmp& QueryPoint::getDmp() {
	return internalDmp;
}

void QueryPoint::setQueryPoint(arma::vec queryPoint) {
	this->queryPoint = queryPoint;
}

void QueryPoint::setDmp(Dmp dmp) {
	this->internalDmp = dmp;
}
