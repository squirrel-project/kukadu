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

    arma::vec t;
    std::vector<arma::vec> y;

public:

    ControllerResult(arma::vec t, std::vector<arma::vec> ys);

    arma::vec getTimes();
    std::vector<arma::vec> getYs();

};



#endif // CONTROLLER_RESULT_H
