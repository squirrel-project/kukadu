#include <cstdio>
#include <iostream>
#include <fstream>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <queue>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_poly.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <vector>
#include <wordexp.h>
#include <RedundantKin.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include "../include/kukadu.h"
#include "../src/utils/gnuplot-cpp/gnuplot_i.hpp"

#define DOSIMULATION 1

using namespace std;
using namespace arma;

void switch2dQueryPoint();
void testPerformance(OrocosControlQueue* queue);
void testHumanoidsGrasping(OrocosControlQueue* queue);
void testHumanoidsArtificialData(ControlQueue* queue);
void testHumanoidsPouring(OrocosControlQueue* simulationQueue, OrocosControlQueue* executionQueue);
void testSegmentationArtificialData(ControlQueue* queue);

Gnuplot* g1 = NULL;
double as = 0.2;
double handVelocity = 20.0;
double tolAbsErr = 1e-1;
double tolRelErr = 1e-1;
DictionaryGeneralizer* dmpGen = NULL;
thread* switchThr = NULL;

double ax = 0.1;
double az = 48.0;
double bz = (az - 1) / 4;
double tau = 0.8;

int kukaStepWaitTime = 14 * 1e3;
double dmpStepSize = kukaStepWaitTime * 1e-6;

// constant for phase stopping
double ac = 10;

int main(int argc, char** args) {

    int mode = 2;

    if(mode == 1) {

        ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
        OrocosControlQueue* queue = NULL;
        if(DOSIMULATION)
            queue = new OrocosControlQueue(argc, args, kukaStepWaitTime, "simulation", "left_arm", *node);
        else
            queue = new OrocosControlQueue(argc, args, kukaStepWaitTime, "real", "left_arm", *node);

        queue->startQueueThread();
        queue->switchMode(10);

        testPerformance(queue);

    } else if (mode == 2) {

//        ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
        PlottingControlQueue* queue = NULL;
        queue = new PlottingControlQueue(1, kukaStepWaitTime);

        queue->startQueueThread();
        queue->switchMode(10);

        testHumanoidsArtificialData(queue);

    } else if (mode == 3) {

        ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
        OrocosControlQueue* queue = NULL;
        if(DOSIMULATION)
            queue = new OrocosControlQueue(argc, args, kukaStepWaitTime, "simulation", "right_arm", *node);
        else
            queue = new OrocosControlQueue(argc, args, kukaStepWaitTime, "real", "right_arm", *node);

        queue->startQueueThread();
        queue->switchMode(10);

        testHumanoidsGrasping(queue);

    } else if(mode == 4) {

        ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
        OrocosControlQueue* simulationQueue = NULL;
        OrocosControlQueue* executionQueue = NULL;

        simulationQueue = new OrocosControlQueue(argc, args, kukaStepWaitTime, "simulation", "left_arm", *node);
        //executionQueue == NULL;
        executionQueue = new OrocosControlQueue(argc, args, kukaStepWaitTime, "simulation", "left_arm", *node);

        simulationQueue->startQueueThread();
        simulationQueue->switchMode(10);

        if(executionQueue != NULL) {
            executionQueue->startQueueThread();
            executionQueue->switchMode(10);
        }

        testHumanoidsPouring(simulationQueue, executionQueue);

    } else if(mode == 5) {

        ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
        PlottingControlQueue* queue = NULL;
        queue = new PlottingControlQueue(1, kukaStepWaitTime);

        queue->startQueueThread();
        queue->switchMode(10);

        testSegmentationArtificialData(queue);

    } else if(mode == 6) {

        string cfFile = "$KUKADU_HOME/movements/humanoids_2014/pouring_gries_eval_traj/traj_11_234.txt";
        string cf2File = "$KUKADU_HOME/movements/humanoids_2014/pouring_gries/traj_9_144.txt";
        DmpRewardComputer rwc(resolvePath(cfFile), az, bz, dmpStepSize, 7);

        t_executor_res res = executeDemo(new PlottingControlQueue(7, dmpStepSize), resolvePath(cf2File), 0, az, bz, 0, dmpStepSize);
        double cost = rwc.computeCost(res);
        cout << "(humanoids2014) cost for comparing same trajectory: " << cost << endl;

    }
    getch();

	return 0;

}

void testPerformance(OrocosControlQueue* queue) {

    string execFile = "$KUKADU_HOME/movements/humanoids_2014/pouring_gries/traj_9_144.txt";
    //string execFile = "$KUKADU_HOME/movements/humanoids_2014/pouring_gries/traj_9_228.txt";
    //string execFile = "$KUKADU_HOME/movements/humanoids_2014/pouring_gries/traj_9_364.txt";
    //string execFile = "$KUKADU_HOME/movements/humanoids_2014/pouring_gries/traj_9_480.txt";
    //string execFile = "$KUKADU_HOME/movements/humanoids_2014/pouring_gries/traj_11_157.txt";
    //string execFile = "$KUKADU_HOME/movements/humanoids_2014/pouring_gries/traj_11_234.txt";
    //string execFile = "$KUKADU_HOME/movements/humanoids_2014/pouring_gries/traj_11_370.txt";
    //string execFile = "$KUKADU_HOME/movements/humanoids_2014/pouring_gries/traj_11_486.txt";
    //string execFile = "$KUKADU_HOME/movements/humanoids_2014/pouring_gries/traj_13_164.txt";
    //string execFile = "$KUKADU_HOME/movements/humanoids_2014/pouring_gries/traj_13_293.txt";
    //string execFile = "$KUKADU_HOME/movements/humanoids_2014/pouring_gries/traj_13_381.txt";
    //string execFile = "$KUKADU_HOME/movements/humanoids_2014/pouring_gries/traj_13_498.txt";

    t_executor_res demoRes = executeDemo(queue, resolvePath(execFile), 0, az, bz, 0);

}

/* switch query point for pouring */
void switch2dQueryPoint() {

    vec qp(2);
    qp(0) = 9;
    qp(1) = 300;

//    switchedTo = qp;

    while(dmpGen->getCurrentTime() < 6.0) {}

    for(int i = 0; i < 9; ++i) {

        sleep(1);
        qp(0) = qp(0) + 0.35;
        cout << "switching execution to position " << qp(0) << endl;
        dmpGen->switchQueryPoint(qp);

    }

}

void testHumanoidsArtificialData(ControlQueue* queue) {

    // for different trajectories, you have to change the reward computer (not the gaussian computer)
    std::string inDir = resolvePath("$KUKADU_HOME/movements/humanoids_2014/pouring_gries/");

    vector<double> irosmys = {0, 1, 2, 3, 4, 5};
    vector<double> irossigmas = {0.3, 0.8};

    vec timeCenters(1); timeCenters(0) = 2.0;
    //vec timeCenters(2); timeCenters(0) = 2.0; timeCenters(1) = 5.0;
    //vec timeCenters(3); timeCenters(0) = 1.0; timeCenters(1) = 2.0; timeCenters(1) = 5.0;

    vector<double> rlExploreSigmas = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    // 2 * number of parameters (http://www.scholarpedia.org/article/Policy_gradient_methods#Finite-difference_Methods)
    int rolloutsPerUpdate = 2 * 4 * timeCenters.n_elem;
    int importanceSamplingCount = 5;

    double alpham = 1.0;

    int columns = 8;

    vec trajMetricWeights(7);
    trajMetricWeights.fill(1.0);

    tau = 5.0;
    float tmp = 0.1;
    double ax = -log(tmp) / tau / tau;
    double relativeDistanceThresh = 0.4;

    vec newQueryPoint(2);
    newQueryPoint(0) = 11.0;
    newQueryPoint(1) = 234.0;

    // wrong reward computer for real robot trajectory!!!!!!!!!!!!!!
//    GaussianObstacleRewardComputer reward(newQueryPoint(0), 2.0, newQueryPoint(1));

    string cfFile = "$KUKADU_HOME/movements/humanoids_2014/pouring_gries_eval_traj/traj_11_234.txt";
    DmpRewardComputer reward(resolvePath(cfFile), az, bz, dmpStepSize, 7);

    cout << "execute ground truth for (1.6, 7)" << endl;
    t_executor_res opt = reward.getOptimalTraj(0, 28.0, 0);
    cout << "execution done" << endl;

    // speedup testing process by inserting already learned metric result
    mat m(2,2);
    /*
    m(0, 0) = 1.0;
    m(1, 0) = -0.2093;
    m(0, 1) = -0.2093;
    m(1, 1) = 0.0590;
    */

    cout << "(main) creating dictionary generalizer object" << endl;
    DictionaryGeneralizer* dmpGen = new DictionaryGeneralizer(timeCenters, newQueryPoint, queue, queue, inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, trajMetricWeights, relativeDistanceThresh, as, alpham);
    cout << "(main) done" << endl;
//    DictionaryGeneralizer* dmpGen = new DictionaryGeneralizer(timeCenters, newQueryPoint, queue, queue, inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, as, m, relativeDistanceThresh, alpham);

    cout << "(main) initializing trajectory" << endl;
    std::vector<Trajectory*> initTraj;
    initTraj.push_back(dmpGen->getTrajectory());
    cout << "(main) done" << endl;

    cout << "(main) query point to maximize is: " << newQueryPoint << endl;
    //cout << "(mainScrewOrocos) first metric: " << ((LinCombDmp*) dmpGen->getTrajectory())->getMetric().getM() << endl;

    LinCombDmp* lastRollout = NULL;

    cout << "(main) creating reinforcement learning environment" << endl;
    PoWER pow(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, &reward, queue, queue, ac, dmpStepSize, tolAbsErr, tolRelErr);
    cout << "(main) done" << endl;
//    GradientDescent pow(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, &reward, queue, queue, ac, dmpStepSize, tolAbsErr, tolRelErr);

    int plotTimes = 5;
    g1 = new Gnuplot("PoWER demo");
    int i = 0;

    vec initT;
    vector<vec> initY;

    vector<double> lastRewards;
    while( i < 50 ) {

        cout << "(main) performing rollout" << endl;
        pow.performRollout(1, 0);
//        cout << "(main) done" << endl;
        lastRollout = dynamic_cast<LinCombDmp*>(pow.getLastUpdate());
        cout << "(main) retrieving last metric" << endl;
        vector<Mahalanobis> metrics = lastRollout->getMetric();

        //cout << "(mainScrewOrocos) last update metric: " << lastRollout->getMetric().getM() << endl;

        cout << "used metrics: " << endl;
        for(int i = 0; i < metrics.size(); ++i)
            cout << metrics.at(i).getM() << endl;

        if(i == 0) {
            initT = pow.getLastUpdateRes().t;
            initY = pow.getLastUpdateRes().y;
        }


        if( (i % 1) == 0 ) {

            int plotNum = 1;
            for(int plotTraj = 0; plotTraj < plotNum; ++plotTraj) {

                ostringstream convert;   // stream used for the conversion
                convert << plotTraj;

                string title = string("fitted sensor data (joint") + convert.str() + string(")");
                g1->set_style("lines").plot_xy(armadilloToStdVec(opt.t), armadilloToStdVec(opt.y[plotTraj]), "optimal trajectoy");
                g1->set_style("lines").plot_xy(armadilloToStdVec(initT), armadilloToStdVec(initY[plotTraj]), "initial trajectoy");
                g1->set_style("lines").plot_xy(armadilloToStdVec(pow.getLastUpdateRes().t), armadilloToStdVec(pow.getLastUpdateRes().y[plotTraj]), "generalized trajectory");
                g1->showonscreen();

            }

        }

        g1->reset_plot();

        if( (i % 20) == 19)
            g1->remove_tmpfiles();

        ++i;

    }


    cout << "(mainScrewOrocos) execution of trajectory at new position" << endl;

    while( 1 ) {

        cout << "(mainScrewOrocos) first coordinate: " << endl;
        cin >> newQueryPoint(0);
        cout << "(mainScrewOrocos) second coordinate: " << endl;
        cin >> newQueryPoint(1);


        lastRollout = ((LinCombDmp*) dmpGen->getTrajectory());
        lastRollout->setCurrentQueryPoint(newQueryPoint);
        dmpGen->switchQueryPoint(newQueryPoint);

        initTraj.clear();
        initTraj.push_back(lastRollout);

        t_executor_res updateRes = dmpGen->simulateTrajectory();

        GaussianObstacleRewardComputer reward(newQueryPoint(0), 2.0, newQueryPoint(1));

        cout << "execute ground truth for (1.6, 7)" << endl;
        t_executor_res opt = reward.getOptimalTraj(7.0, 0);

        g1 = new Gnuplot("PoWER demo2");
        g1->set_style("points").plot_xy(armadilloToStdVec(updateRes.t), armadilloToStdVec(updateRes.y[0]), "generalized trajectory");
        g1->set_style("lines").plot_xy(armadilloToStdVec(opt.t), armadilloToStdVec(opt.y[0]), "optimal trajectory");
        g1->showonscreen();

        cout << "(mainScrewOrocos) press key to continue" << endl;
        getchar();
        getchar();

        delete g1;

    }

}

void testHumanoidsPouring(OrocosControlQueue* simulationQueue, OrocosControlQueue* executionQueue) {

//    az = 80.0;
//    bz = (az - 1) / 4;

    std::string inDir = resolvePath("$KUKADU_HOME/movements/humanoids_2014/pouring_gries/");
    std::string resFile = resolvePath("$KUKADU_HOME/movements/humanoids_2014/eval_pouring_gries4.txt");

    double alpham = 1.0;

    ControlQueue* raQueue = NULL;
    QuadraticKernel* kern = new QuadraticKernel();

    vector<double> irosmys = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 35.5};
    vector<double> irossigmas = {0.3, 0.8};

    //vector<double> rlExploreSigmas = {0.5, 0.5, 0.5, 0.5};
    vector<double> rlExploreSigmas = {0.02, 0.02, 0.01, 0.02};
    int rolloutsPerUpdate = 4;
    int importanceSamplingCount = 3;

    int columns = 8;

    vec trajMetricWeights(7);
    trajMetricWeights.fill(1.0);

    float tmp = 0.1;
    double relativeDistanceThresh = 0.4;

    vec newQueryPoint(2);
    newQueryPoint(0) = 9;
    newQueryPoint(1) = 300;

    /* query point to reinforce the metric */
    /*
    newQueryPoint(0) = 10;
    newQueryPoint(1) = 300;
    */

    CostComputer* reward = new PouringRewardComputer(newQueryPoint(1));

    // speedup testing process by inserting already learned metric result

    mat m(2,2);
    /*
    // intermediate result
    m(0, 0) = 1.0;
    m(1, 0) = 5.4189e-20;
    m(0, 1) = 5.4189e-20;
    m(1, 1) = 4.5177e-04;
    */

    // metric after rl
    m(0, 0) = 1.0;
    m(1, 0) = 0;
    m(0, 1) = 0;
    m(1, 1) = 0.0007;

    vec timeCenters(1);
    timeCenters(0) = 2.5;

    /*
1.0000  -0.0281
  -0.0281   0.0009
  */

//    DictionaryGeneralizer* dmpGen = new DictionaryGeneralizer(newQueryPoint, raQueue, inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, trajMetricWeights, relativeDistanceThresh, as);
    dmpGen = new DictionaryGeneralizer(timeCenters, newQueryPoint, simulationQueue, executionQueue, inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, as, m, relativeDistanceThresh, alpham);

    std::vector<Trajectory*> initTraj;
    initTraj.push_back(dmpGen->getTrajectory());

    cout << newQueryPoint << endl;
    //cout << "(mainScrewOrocos) first metric: " << ((LinCombDmp*) dmpGen->getTrajectory())->getMetric().getM() << endl;


    LinCombDmp* lastRollout = NULL;
    PoWER pow(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, reward, simulationQueue, executionQueue, ac, dmpStepSize, tolAbsErr, tolRelErr);

    g1 = new Gnuplot("PoWER demo");

    // commented out for testing
    vector<double> lastRewards;

    int i = 0;
    vec initT;
    vector<vec> initY;
    int plotTimes = 5;

    /*
    cout << "(main) performing reinforcement learning" << endl;
    while( true ) {

        pow.performRollout(1, 1);
        lastRollout = dynamic_cast<LinCombDmp*>(pow.getLastUpdate());
        lastRewards.push_back(pow.getLastUpdateReward());

        cout << "(mainScrewOrocos) last update metric: " << lastRollout->getMetric().getM() << endl;
//		cout << lastRollout->getMetric().getM() << endl << endl;

        if(i == 0) {
            initT = pow.getLastUpdateRes().t;
            initY = pow.getLastUpdateRes().y;
        }

    }

    */


    //cout << "reached metric: " << ((LinCombDmp*) dmpGen->getTrajectory())->getMetric().getM() << endl;

    ofstream oFile;
    oFile.open(resFile);

    cout << "(PouringExperiment) execution of trajectory at new position" << endl;

    int count = 0;
    double totalError = 0.0;
    int degFreedom = dmpGen->getTrajectory()->getDegreesOfFreedom();
    arma::vec startingJoints(1);
    while( 1 ) {

        cout << "(PouringExperiment) first coordinate (< 0 for exit): " << endl;
        cin >> newQueryPoint(0);

        if(newQueryPoint(0) < 0)
            break;

        cout << "(PouringExperiment) second coordinate: " << endl;
        cin >> newQueryPoint(1);

//        cout << "(PouringExperiment) insert new value for as: " << endl;
//        cin >> as;

        dmpGen->setAs(as);

        lastRollout = ((LinCombDmp*) dmpGen->getTrajectory());
        lastRollout->setCurrentQueryPoint(newQueryPoint);
        dmpGen->switchQueryPoint(newQueryPoint);

        vec startingPos = dmpGen->getTrajectory()->getStartingPos();
        startingJoints = startingPos;

        initTraj.clear();
        initTraj.push_back(lastRollout);

        simulationQueue->moveJoints(startingJoints);
//        switchThr = new std::thread(switch2dQueryPoint);
        t_executor_res updateRes = dmpGen->simulateTrajectory();
//        switchThr->join();
        simulationQueue->moveJoints(startingJoints);

        char cont = 'n';
        cout << "(main) do you want to execute this trajectory? (y/N) ";
        cin >> cont;

        if(cont == 'y' || cont == 'Y') {

            cout << "(main) executing trial" << endl;

            lastRollout = ((LinCombDmp*) dmpGen->getTrajectory());
            lastRollout->setCurrentQueryPoint(newQueryPoint);
            dmpGen->switchQueryPoint(newQueryPoint);

            executionQueue->moveJoints(startingJoints);
//            switchThr = new std::thread(switch2dQueryPoint);
            dmpGen->executeTrajectory();
//            switchThr->join();

        }

    }

}

void testHumanoidsGrasping(OrocosControlQueue* queue) {

    std::string inDir = resolvePath("$KUKADU_HOME/movements/humanoids_2014/reaching_on_grid/");
    std::string resFile = resolvePath("$KUKADU_HOME/movements/humanoids_2014/eval_reaching.txt");

    ControlQueue* raQueue = NULL;
    QuadraticKernel* kern = new QuadraticKernel();

    double alpham = 1.0;

    vector<double> irosmys = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    vector<double> irossigmas = {0.3, 0.8};

    //vector<double> rlExploreSigmas = {0.5, 0.5, 0.5, 0.5};
    vector<double> rlExploreSigmas = {0.2, 0.2, 0.2, 0.2};
    int rolloutsPerUpdate = 5;
    int importanceSamplingCount = 3;

    int columns = 8;

    vec trajMetricWeights(7);
    trajMetricWeights.fill(1.0);

    tau = 5.0;
    float tmp = 0.1;
    double ax = -log(tmp) / tau / tau;
    double relativeDistanceThresh = 1.0;

    cout << "tau: " << tau << endl;
    cout << "ax: " << ax << endl;

    vec newQueryPoint(2);
    newQueryPoint(0) = 8;
    newQueryPoint(1) = 5;

    // result for (8, 8)
    vector<double> vectorNewQueryPoint = {-0.387412, 1.56013, 1.05285, 1.17488, 0.182785, -0.819336, -0.0848888};

    GraspingRewardComputer reward(vectorNewQueryPoint);

    cout << "execute ground truth for (8, 6) from file " << inDir + "../traj_" + stringFromDouble(newQueryPoint(0)) + "-" + stringFromDouble(newQueryPoint(1)) + ".txt" << endl;
    t_executor_res opt = executeDemo(NULL, inDir + "../traj_" + stringFromDouble(newQueryPoint(0)) + "-" + stringFromDouble(newQueryPoint(1)) + ".txt", 1, az, bz, 0);
    // t_executor_res opt = executeDemo(queue, inDir + "../traj_" + stringFromDouble(newQueryPoint(0)) + "-" + stringFromDouble(newQueryPoint(1)) + ".txt", 0, az, bz, 1);

    // speedup testing process by inserting already learned metric result
    mat m(2,2);
    m(0, 0) = 1.0;
    m(1, 0) = -1.3435;
    m(0, 1) = -1.3435;
    m(1, 1) = 1.8495;

    /*
    1.0000  -1.3435
    -1.3435   1.8495
  */
    vec timeCenters(1);
    timeCenters(0) = 2.5;

//    DictionaryGeneralizer* dmpGen = new DictionaryGeneralizer(newQueryPoint, raQueue, inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, trajMetricWeights, relativeDistanceThresh, as, alpham);
    DictionaryGeneralizer* dmpGen = new DictionaryGeneralizer(timeCenters, newQueryPoint, queue, NULL, inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, as, m, relativeDistanceThresh, alpham);

    std::vector<Trajectory*> initTraj;
    initTraj.push_back(dmpGen->getTrajectory());

    cout << newQueryPoint << endl;
    //cout << "(mainScrewOrocos) first metric: " << ((LinCombDmp*) dmpGen->getTrajectory())->getMetric().getM() << endl;

    LinCombDmp* lastRollout = NULL;

//    PoWER pow(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, &reward, NULL, ac, dmpStepSize, tolAbsErr, tolRelErr);
    GradientDescent pow(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, &reward, NULL, NULL, ac, dmpStepSize, tolAbsErr, tolRelErr);
    g1 = new Gnuplot("PoWER demo");

    /*
    // commented out for testing
    vector<double> lastRewards;

    int i = 0;

    vec initT;
    vector<vec> initY;
    int plotTimes = 5;

    while( i < 8 ) {

        pow.performRollout(1, 0);
        lastRollout = dynamic_cast<LinCombDmp*>(pow.getLastUpdate());
        lastRewards.push_back(pow.getLastUpdateReward());
        if(lastRewards.size() > 3)
            lastRewards.erase(lastRewards.begin());

        cout << "(mainScrewOrocos) last update metric: " << lastRollout->getMetric().getM() << endl;
//		cout << lastRollout->getMetric().getM() << endl << endl;

        if(i == 0) {
            initT = pow.getLastUpdateRes().t;
            initY = pow.getLastUpdateRes().y;
        }


        if( (i % 1) == 0 ) {

            cout << "(mainScrewOrocos) already done " << i << " iterations" << endl;

            int plotNum = 1;
            for(int plotTraj = 0; plotTraj < plotNum; ++plotTraj) {

                ostringstream convert;   // stream used for the conversion
                convert << plotTraj;

                string title = string("fitted sensor data (joint") + convert.str() + string(")");
                g1->set_style("lines").plot_xy(armadilloToStdVec(opt.t), armadilloToStdVec(opt.y[plotTraj]), "optimal trajectoy");
                g1->set_style("lines").plot_xy(armadilloToStdVec(initT), armadilloToStdVec(initY[plotTraj]), "initial trajectoy");
                g1->set_style("lines").plot_xy(armadilloToStdVec(pow.getLastUpdateRes().t), armadilloToStdVec(pow.getLastUpdateRes().y[plotTraj]), "generalized trajectory");
                g1->showonscreen();

            }

        }

        g1->reset_plot();

        if( (i % 20) == 19)
            g1->remove_tmpfiles();

        ++i;

    }
*/
    // new optimal metric:
    m(0, 0) = 1.0;
    m(1, 0) = -0.1698;
    m(0, 1) = -0.1698;
    m(1, 1) = 1.1672;

    Mahalanobis metric(m);
    vector<vec> metricCoeffs = {metric.getCoefficients()};
    dmpGen->getTrajectory()->setCoefficients(metricCoeffs);

    ofstream oFile;
    oFile.open(resFile);

    cout << "(mainScrewOrocos) execution of trajectory at new position" << endl;

    int count = 0;
    double totalError = 0.0;
    while( 1 ) {

        cout << "(mainScrewOrocos) first coordinate (< 0 for exit): " << endl;
        cin >> newQueryPoint(0);

        if(newQueryPoint(0) < 0)
            break;

        cout << "(mainScrewOrocos) second coordinate: " << endl;
        cin >> newQueryPoint(1);


        lastRollout = ((LinCombDmp*) dmpGen->getTrajectory());
        lastRollout->setCurrentQueryPoint(newQueryPoint);
        dmpGen->switchQueryPoint(newQueryPoint);

        initTraj.clear();
        initTraj.push_back(lastRollout);

        t_executor_res updateRes = dmpGen->executeTrajectory();

//        g1 = new Gnuplot("PoWER demo2");
//        g1->set_style("points").plot_xy(armadilloToStdVec(updateRes.t), armadilloToStdVec(updateRes.y[0]), "generalized trajectory");
//        g1->set_style("lines").plot_xy(armadilloToStdVec(opt.t), armadilloToStdVec(opt.y[0]), "optimal trajectory");
//        g1->showonscreen();

        double mesError = 0.0;
        cout << "(humanoids2014) please insert the measured error: ";
        cin >> mesError;

        totalError += mesError;
        ++count;

        oFile << "(" << newQueryPoint(0) << ", " << newQueryPoint(1) << "): " << mesError << endl;

//        cout << "(mainScrewOrocos) press key to continue" << endl;
//        getchar();
//        getchar();

//        delete g1;

    }

    oFile << "average error: " << (totalError / count) << endl;

}

void testSegmentationArtificialData(ControlQueue* queue) {

    // for different trajectories, you have to change the reward computer (not the gaussian computer)
    // std::string inDir = resolvePath("$KUKADU_HOME/movements/iros2014/2d_extended_gen/");
    std::string inDir = resolvePath("$KUKADU_HOME/movements/iros2014/artificial_complete/");

    vector<double> irosmys = {0, 1};
    vector<double> irossigmas = {0.3, 0.8};

    //vec timeCenters(1); timeCenters(0) = 1.0;
    vec timeCenters(2); timeCenters(0) = 1.0; timeCenters(1) = 5.0;
    //vec timeCenters(3); timeCenters(1) = 1.0; timeCenters(1) = 2.5; timeCenters(2) = 4.0;

    vector<double> rlExploreSigmas = {0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
    // 2 * number of parameters (http://www.scholarpedia.org/article/Policy_gradient_methods#Finite-difference_Methods)
    int rolloutsPerUpdate = 2 * 4 * timeCenters.n_elem;
    int importanceSamplingCount = 5;

    double alpham = 1.0;

    int columns = 2;

    vec trajMetricWeights(1);
    trajMetricWeights.fill(1.0);

    tau = 5.0;
    float tmp = 0.1;
    double ax = -log(tmp) / tau / tau;
    double relativeDistanceThresh = 0.6;

    vec newQueryPoint(2);
    newQueryPoint(0) = 1.6;
    newQueryPoint(1) = 6.5;

    SegmentationTestingRewardComputer reward(6, 2.5, 1);

    cout << "execute ground truth for (6, 2.5)" << endl;
    t_executor_res opt = reward.getOptimalTraj(0, 5);

    // speedup testing process by inserting already learned metric result
    mat m(2,2);
    m(0, 0) = 1.0;
    m(1, 0) = 0;
    m(0, 1) = 0;
    m(1, 1) = 1;

//    DictionaryGeneralizer* dmpGen = new DictionaryGeneralizer(timeCenters, newQueryPoint, queue, queue, inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, trajMetricWeights, relativeDistanceThresh, as, alpham);
    DictionaryGeneralizer* dmpGen = new DictionaryGeneralizer(timeCenters, newQueryPoint, queue, queue, inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, as, m, relativeDistanceThresh, alpham);

    std::vector<Trajectory*> initTraj;
    initTraj.push_back(dmpGen->getTrajectory());

    cout << newQueryPoint << endl;
    //cout << "(mainScrewOrocos) first metric: " << ((LinCombDmp*) dmpGen->getTrajectory())->getMetric().getM() << endl;

    LinCombDmp* lastRollout = NULL;

    PoWER pow(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, &reward, queue, queue, ac, dmpStepSize, tolAbsErr, tolRelErr);
//    GradientDescent pow(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, &reward, queue, queue, ac, dmpStepSize, tolAbsErr, tolRelErr);

    int plotTimes = 5;
    g1 = new Gnuplot("PoWER demo");
    int i = 0;

    vec initT;
    vector<vec> initY;

    vector<double> lastRewards;
    while( i < 50 ) {

        pow.performRollout(1, 0);
        lastRollout = dynamic_cast<LinCombDmp*>(pow.getLastUpdate());
        vector<Mahalanobis> metrics = lastRollout->getMetric();

        //cout << "(mainScrewOrocos) last update metric: " << lastRollout->getMetric().getM() << endl;

        cout << "used metrics: " << endl;
        for(int i = 0; i < metrics.size(); ++i)
            cout << metrics.at(i).getM() << endl;

        if(i == 0) {
            initT = pow.getLastUpdateRes().t;
            initY = pow.getLastUpdateRes().y;
        }


        if( (i % 1) == 0 ) {

            int plotNum = 1;
            for(int plotTraj = 0; plotTraj < plotNum; ++plotTraj) {

                ostringstream convert;   // stream used for the conversion
                convert << plotTraj;

                g1->set_xlabel("time");
                g1->set_ylabel("y");
                string title = string("fitted sensor data (joint") + convert.str() + string(")");
                g1->set_style("lines lw 3").plot_xy(armadilloToStdVec(opt.t), armadilloToStdVec(opt.y[plotTraj]), "optimal trajectoy");
                g1->set_style("lines lw 3").plot_xy(armadilloToStdVec(initT), armadilloToStdVec(initY[plotTraj]), "initial trajectoy");
                g1->set_style("lines lw 3").plot_xy(armadilloToStdVec(pow.getLastUpdateRes().t), armadilloToStdVec(pow.getLastUpdateRes().y[plotTraj]), "generalized trajectory");
                g1->showonscreen();

            }

        }

        g1->reset_plot();

        if( (i % 20) == 19)
            g1->remove_tmpfiles();

        ++i;

    }


    cout << "(mainScrewOrocos) execution of trajectory at new position" << endl;

    while( 1 ) {

        cout << "(mainScrewOrocos) first coordinate: " << endl;
        cin >> newQueryPoint(0);
        cout << "(mainScrewOrocos) second coordinate: " << endl;
        cin >> newQueryPoint(1);


        lastRollout = ((LinCombDmp*) dmpGen->getTrajectory());
        lastRollout->setCurrentQueryPoint(newQueryPoint);
        dmpGen->switchQueryPoint(newQueryPoint);

        initTraj.clear();
        initTraj.push_back(lastRollout);

        t_executor_res updateRes = dmpGen->simulateTrajectory();

        GaussianObstacleRewardComputer reward(newQueryPoint(0), 2.0, newQueryPoint(1));

        cout << "execute ground truth for (1.6, 7)" << endl;
        t_executor_res opt = reward.getOptimalTraj(7.0, 0);

        g1 = new Gnuplot("PoWER demo2");
        g1->set_style("points").plot_xy(armadilloToStdVec(updateRes.t), armadilloToStdVec(updateRes.y[0]), "generalized trajectory");
        g1->set_style("lines").plot_xy(armadilloToStdVec(opt.t), armadilloToStdVec(opt.y[0]), "optimal trajectory");
        g1->showonscreen();

        cout << "(mainScrewOrocos) press key to continue" << endl;
        getchar();
        getchar();

        delete g1;

    }


}
