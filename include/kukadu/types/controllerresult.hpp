#ifndef CONTROLLER_RESULT_H
#define CONTROLLER_RESULT_H

#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <wordexp.h>
#include <memory>
#include <armadillo>

#ifndef USEBOOST
#include <tuple>
#include <kukadu/learning/projective_simulation/core/clip.hpp>
#endif

namespace kukadu {

    class ControllerResult {

    private:

        bool success;

        arma::vec t;
        std::vector<arma::vec> y;

    public:

        ControllerResult(arma::vec t, std::vector<arma::vec> ys, bool success);

        arma::vec getTimes();
        std::vector<arma::vec> getYs();

        void setSuccess(bool success);

        bool getSuccess();

        // needs one virtual method in order to make ControllerResult polymorphic (required for dynamic pointer cast)
        virtual ~ControllerResult() { }

    };

#ifndef USEBOOST

    class HapticControllerResult : public ControllerResult {

    private:

        bool bored;

        std::vector<int> walkedPath;

        std::vector<std::pair<double, double> > meanAndVar;

        KUKADU_SHARED_PTR<std::tuple<double, KUKADU_SHARED_PTR<kukadu::Clip>, std::vector<KUKADU_SHARED_PTR<kukadu::Clip> > > > environmentTransition;

    public:

        HapticControllerResult(arma::vec t, std::vector<arma::vec> ys, bool success, bool bored, std::vector<int> walkedPath, KUKADU_SHARED_PTR<std::tuple<double, KUKADU_SHARED_PTR<kukadu::Clip>, std::vector<KUKADU_SHARED_PTR<kukadu::Clip> > > > environmentTransition);

        bool wasBored();

        std::vector<int> getWalkedPath();

        std::vector<std::pair<double, double> > getMeanAndVar();

        void setEntropyMeanAndVariance(std::vector<std::pair<double, double> > meanAndVar);

    };

#endif

}

#endif // CONTROLLER_RESULT_H
