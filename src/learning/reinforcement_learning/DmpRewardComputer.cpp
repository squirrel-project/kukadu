#include "DmpRewardComputer.h"

using namespace std;
using namespace arma;

DmpRewardComputer::DmpRewardComputer(string file, double az, double bz, double timeStep, int degOfFreedom, double tmax) : TrajectoryBasedReward(degOfFreedom, tmax) {

    this->file = file;
    this->az = az;
    this->bz = bz;
    this->timeStep = timeStep;
    std::shared_ptr<ControlQueue> pcq = std::shared_ptr<ControlQueue>(new PlottingControlQueue(degOfFreedom, timeStep));

    cout << "(DmpRewardComputer) starting execution of sample trajectory with timeStep size " << timeStep << endl;
    executionResult = executeDemo(pcq, file, az, bz, 0);

    /*
    cout << "(DmpRewardComputer) performing binary tree search" << endl;
    int idx = binaryTimeSearch(executionResult.t, 0.51);

    cout << "(DmpRewardComputer) found t at index " << idx << endl;
    cout << "(DmpRewardComputer) value is " << executionResult.t(idx) << " and " << executionResult.t(idx + 1) << endl;
    */

}

arma::vec DmpRewardComputer::computeFun(double t) {

    vec time = executionResult.t;
    vec retVec(executionResult.y.size());

//    cout << "(DmpRewardComputer) dim: " << executionResult.y.size() << endl;
    if(t >= time(time.n_elem - 1)) {
        for(int i = 0; i < retVec.n_elem; ++i)
                retVec(i) = executionResult.y.at(i)(time.n_elem - 1);

    } else {

        int tIdx = binaryTimeSearch(time, t);
        double firstT = time(tIdx);
        double secondT = time(tIdx + 1);
        double firstDist = timeStep - t + firstT;
        double secondDist = timeStep - secondT + t;
        for(int i = 0; i < retVec.n_elem; ++i)
                retVec(i) = (firstDist * executionResult.y.at(i)(tIdx) + secondDist * executionResult.y.at(i)(tIdx + 1)) / timeStep;

    }

    return retVec;

}

int DmpRewardComputer::binaryTimeSearch(arma::vec times, double t) {

    int start = 0;
    int end = times.n_elem - 1;
    int middle = (start + end) / 2;

    if(t >= times(end))
        return end;
    else if(t <= times(start))
        return start;

    while(start != end) {

        if(t >= times(middle) && t < (times(middle) + timeStep))
            return middle;

        if(times(middle) > t)
            end = middle;
        else
            start = middle;

        middle = (start + end) / 2;


    }

    return -1;

}
