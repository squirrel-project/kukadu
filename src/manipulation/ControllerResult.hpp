#ifndef CONTROLLER_RESULT_H
#define CONTROLLER_RESULT_H

#include <cstdio>
#include <iostream>
#include <fstream>
#include <thread>
#include <string>
#include <vector>
#include <wordexp.h>
#include <memory>
#include <armadillo>

class ControllerResult {

private:

    bool success;

    arma::vec t;
    std::vector<arma::vec> y;

    void construct(arma::vec t, std::vector<arma::vec> ys, bool success);

public:

    ControllerResult(arma::vec t, std::vector<arma::vec> ys);
    ControllerResult(arma::vec t, std::vector<arma::vec> ys, bool success);

    arma::vec getTimes();
    std::vector<arma::vec> getYs();

    void setSuccess(bool success);
    bool getSuccess();

};



#endif // CONTROLLER_RESULT_H
