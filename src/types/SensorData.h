#ifndef SENSORDATA
#define SENSORDATA

#include <vector>
#include <string>
#include <armadillo>
#include <utility>

#include "../robot/SensorStorage.h"

class SensorData {

private:

    std::vector<std::string> labels;
    arma::mat values;

    void readStorage(std::string file);

public:

    SensorData(std::vector<std::string> labels, arma::mat values);
    SensorData(std::string file);

    int labelExists(std::string label);
    arma::vec getDataByIdx(int idx);
    arma::vec getDataByLabel(std::string label);
    arma::vec getTime();
    arma::mat getRange(std::vector<std::string> indexes);
    arma::mat getRange(std::vector<int> indexes);
    arma::mat getRange(int startIdx, int endIdx);


};

#endif
