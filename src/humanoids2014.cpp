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

void testPerformance(OrocosControlQueue* queue);
void testIROSGrasping(OrocosControlQueue* queue);
std::string resolvePath(std::string path);

Gnuplot* g1 = NULL;
double as = 1.0;
double handVelocity = 20.0;
double tolAbsErr = 1e-3;
double tolRelErr = 1e-3;

double ax = 0.1;
double az = 48.0;
double bz = (az - 1) / 4;
double tau = 0.8;

int kukaStepWaitTime = 1.8 * 1e4;
double dmpStepSize = kukaStepWaitTime * 1e-6;

// constant for phase stopping
double ac = 10;

// before exeuction in simulator: rostopic pub /simulation/right_arm/joint_control/set_velocity_limit std_msgs/Float32 100

std::string resolvePath(std::string path) {

    wordexp_t p;
    wordexp(path.c_str(), &p, 0 );
    char** w = p.we_wordv;
    string ret = string(*w);
    wordfree( &p );

    return ret;

}

int main(int argc, char** args) {

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    OrocosControlQueue* queue = new OrocosControlQueue(argc, args, kukaStepWaitTime, "simulation", "right_arm", *node);
    queue->startQueueThread();

//    testPerformance(queue);
    testIROSGrasping(queue);
    getch();

	return 0;

}

void testPerformance(OrocosControlQueue* queue) {

    t_executor_res demoRes = executeDemo(queue, resolvePath("$KUKADU_HOME/movements/iros2014/real_robot_2d_pickup/traj_10-5.txt"),
                               0, az, bz, 1);

}

void testIROSGrasping(OrocosControlQueue* queue) {

    // for different trajectories, you have to change the reward computer (not the gaussian computer)
    std::string inDir = resolvePath("$KUKADU_HOME/movements/iros2014/2d_extended_gen/");
    QuadraticKernel* kern = new QuadraticKernel();

    vector<double> irosmys = {0, 1, 2, 3, 4, 5, 6, 7};
    vector<double> irossigmas = {0.3, 0.8};

    vector<double> rlExploreSigmas = {0.5, 0.5, 0.5, 0.5};
    //vector<double> rlExploreSigmas = {0.2, 0.2, 0.2, 0.2};
    int rolloutsPerUpdate = 20;
    int importanceSamplingCount = 5;

    int columns = 8;

    vec trajMetricWeights(7);
    trajMetricWeights.fill(1.0);

    tau = 5.0;
    float tmp = 0.1;
    double ax = -log(tmp) / tau / tau;
    double relativeDistanceThresh = 0.4;

    vec newQueryPoint(2);
    newQueryPoint(0) = 1.6;
    newQueryPoint(1) = 6.5;

    // wrong reward computer for real robot trajectory!!!!!!!!!!!!!!
    GaussianObstacleRewardComputer reward(newQueryPoint(0), 2.0, newQueryPoint(1));

    cout << "execute ground truth for (1.6, 7)" << endl;
    t_executor_res opt = reward.getOptimalTraj(7.0);

    // speedup testing process by inserting already learned metric result
    mat m(2,2);
    m(0, 0) = 1.0;
    m(1, 0) = -0.2093;
    m(0, 1) = -0.2093;
    m(1, 1) = 0.0590;

//    DictionaryGeneralizer* dmpGen = new DictionaryGeneralizer(newQueryPoint, NULL, inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, trajMetricWeights, relativeDistanceThresh, as);
    DictionaryGeneralizer* dmpGen = new DictionaryGeneralizer(newQueryPoint, NULL, inDir, columns - 1, irosmys, irossigmas, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, as, m, relativeDistanceThresh);

    std::vector<Trajectory*> initTraj;
    initTraj.push_back(dmpGen->getTrajectory());

    cout << newQueryPoint << endl;
    cout << "(mainScrewOrocos) first metric: " << ((LinCombDmp*) dmpGen->getTrajectory())->getMetric().getM() << endl;

    LinCombDmp* lastRollout = NULL;

//    PoWER pow(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, &reward, NULL, ac, dmpStepSize, tolAbsErr, tolRelErr);
    GradientDescent pow(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, &reward, NULL, ac, dmpStepSize, tolAbsErr, tolRelErr);

    int plotTimes = 5;
    g1 = new Gnuplot("PoWER demo");
    int i = 0;

    vec initT;
    vector<vec> initY;

    vector<double> lastRewards;
    while( i < 50 ) {

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

        g1 = new Gnuplot("PoWER demo2");
        g1->set_style("points").plot_xy(armadilloToStdVec(updateRes.t), armadilloToStdVec(updateRes.y[0]), "generalized trajectory");
    //	g1->set_style("lines").plot_xy(armadilloToStdVec(opt.t), armadilloToStdVec(opt.y[0]), "optimal trajectory");
        g1->showonscreen();

        cout << "(mainScrewOrocos) press key to continue" << endl;
        getchar();
        getchar();

        delete g1;

    }

}
