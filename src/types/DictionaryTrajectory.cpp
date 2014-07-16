#include "DictionaryTrajectory.h"

using namespace std;
using namespace arma;

DictionaryTrajectory::DictionaryTrajectory(int degOfFreedom, std::string baseFolder, std::vector<DMPBase> baseDef, double az, double bz) : Trajectory() {

	this->degOfFreedom = degOfFreedom;
	this->baseFolder = baseFolder;
	vector<string> files = getFilesInDirectory(baseFolder);
	queryFiles = sortPrefix(files, "query");
	trajFiles = sortPrefix(files, "traj");
    dmpFiles = sortPrefix(files, "dmp");

	queryPoints = mapFiles(queryFiles, trajFiles, "query", "traj");
    TrajectoryDMPLearner* dmpLearner;
	
	this->baseDef = baseDef;
	
    vector<mat> jointsVec;
    double tMax = 0.0;
	for(int i = 0; i < queryPoints.size(); ++i) {

        mat joints = readMovements((string(baseFolder) + string(queryPoints.at(i).getFileDataPath())).c_str(), degOfFreedom + 1);
		queryPoints.at(i).setQueryPoint(readQuery(string(baseFolder) + string(queryPoints.at(i).getFileQueryPath())));
        jointsVec.push_back(joints);
        double currentTMax = joints(joints.n_rows - 1, 0);

        if(tMax < currentTMax)
                tMax = currentTMax;
		
    }

    mat joints = jointsVec.at(0);
    double tau = 0.8;
    double ax = -log((float) 0.1) / joints(joints.n_rows - 1, 0) / tau;

    for(int i = 0; i < jointsVec.size(); ++i) {

        mat joints = jointsVec.at(i);
        int maxRows = joints.n_rows;
        joints = fillTrajectoryMatrix(joints, tMax);
        dmpLearner = new TrajectoryDMPLearner(baseDef, tau, az, bz, ax, joints, degOfFreedom);
        Dmp learnedDmps = dmpLearner->fitTrajectories();
        queryPoints.at(i).setDmp(learnedDmps);

        cout << "(DMPGeneralizer) goals for query point [" << queryPoints.at(i).getQueryPoint().t() << "]" << endl << "\t [";
        cout << queryPoints.at(i).getDmp().getG().t() << "]" << endl;

        delete dmpLearner;

    }

}

std::vector<arma::vec> DictionaryTrajectory::getCoefficients() {
	return coefficients;
}

void DictionaryTrajectory::setCoefficients(std::vector<arma::vec> coeffs) {
	coefficients = coeffs;
}

DictionaryTrajectory::DictionaryTrajectory(const DictionaryTrajectory& copy) : Trajectory(copy) {
	
	this->degOfFreedom = copy.degOfFreedom;
	this->baseFolder = copy.baseFolder;
	this->baseDef = copy.baseDef;
	
	this->files = copy.files;
	this->queryFiles = copy.queryFiles;
	this->trajFiles = copy.trajFiles;
	this->queryPoints = copy.queryPoints;
	
}

DictionaryTrajectory::DictionaryTrajectory() {
}

int DictionaryTrajectory::getDegreesOfFreedom() const {
	return degOfFreedom;
}

// TODO: implement == operator
int DictionaryTrajectory::operator==(DictionaryTrajectory const& comp) const {
	string errStr = "(DictionaryTrajectory) == operator not implemted yet";
	cerr << errStr << endl;
	throw errStr;
}

vector<QueryPoint> DictionaryTrajectory::mapFiles(vector<string> queryFiles, vector<string> trajFiles, string prefix1, string prefix2) {
	
	vector<QueryPoint> ret;
	
	int prefix1Size = prefix1.size();
	int prefix2Size = prefix2.size();
	int querySize = queryFiles.size();
	int trajSize = trajFiles.size();
	
	for(int i = 0; i < querySize; ++i) {
		string currentQueryFile = string(queryFiles.at(i));
		string queryAppendix = currentQueryFile.substr(prefix1Size, currentQueryFile.size() - 1);
		for(int j = 0; j < trajSize; ++j) {
			string currentTrajFile = string(trajFiles.at(j));
			string trajAppendix = currentTrajFile.substr(prefix2Size, currentTrajFile.size() - 1);
			if(!queryAppendix.compare(trajAppendix)) {
				QueryPoint toAdd(queryFiles.at(i), trajFiles.at(j), Dmp(), vec());
				ret.push_back(toAdd);
			}
		}
	}
	
	return ret;
	
}

double DictionaryTrajectory::getTmax() {
	return queryPoints.at(0).getDmp().getTmax();
}

std::vector<QueryPoint> DictionaryTrajectory::getQueryPoints() {
	return queryPoints;
}

arma::vec DictionaryTrajectory::getStartingPos() {
	
	string errStr = "(DictionaryTrajectory) no starting position set yet";
	
	if(startingPos.n_elem > 0)
		return startingPos;
	
	cerr << errStr << endl;
	throw errStr;
		
}

Trajectory* DictionaryTrajectory::copy() {
	return new DictionaryTrajectory(*this);
}
