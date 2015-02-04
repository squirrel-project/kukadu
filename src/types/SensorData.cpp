#include "SensorData.h"

using namespace std;
using namespace arma;

SensorData::SensorData(std::vector<std::string> labels, arma::mat values) {

    this->labels = labels;
    this->values = values;

}

SensorData::SensorData(std::string file) {

    readStorage(file);

}

void SensorData::readStorage(std::string file) {

    pair<vector<string>, mat> storage = readSensorStorage(file);
    labels = storage.first;
    values = storage.second;

}

int SensorData::labelExists(std::string label) {

    for(int i = 0; i < labels.size(); ++i) {
        if(!label.compare(labels.at(i)))
            return i;
    }

    return -1;

}

arma::vec SensorData::getDataByIdx(int idx) {

    return vec(values.col(idx));

}

arma::vec SensorData::getTime() {

    return getDataByLabel("time");

}

arma::vec SensorData::getDataByLabel(std::string label) {

    return getDataByIdx(labelExists(label));

}

arma::mat SensorData::getRange(std::vector<std::string> indexes) {

    vector<int> intIndexes;
    for(int i = 0; i < indexes.size(); ++i) {

        int idx = labelExists(indexes.at(i));
        if(idx >= 0)
            intIndexes.push_back(idx);
        else
            return mat(1, 1);

    }

    return getRange(intIndexes);

}

arma::mat SensorData::getRange(std::vector<int> indexes) {

    mat retMat = getDataByIdx(indexes.at(0));
    for(int i = 1; i < indexes.size(); ++i) {
        vec currVec = getDataByIdx(indexes.at(i));
        retMat = join_rows(retMat, currVec);
    }

    return retMat;

}

arma::mat SensorData::getRange(int startIdx, int endIdx) {

    vector<int> rangeVec;
    for(; startIdx < endIdx; ++startIdx)
        rangeVec.push_back(startIdx);

    return getRange(rangeVec);

}
