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

#define DOSIMULATION 0

using namespace std;
using namespace arma;

std::string resolvePath(std::string path);
void testPower(ros::NodeHandle* node);

Gnuplot* g1 = NULL;
double as = 0.2;
double handVelocity = 20.0;
double tolAbsErr = 1e-1;
double tolRelErr = 1e-1;

double ax = 0.1;
double az = 48.0;
double bz = (az - 1) / 4;
double tau = 0.8;

int kukaStepWaitTime = 14 * 1e3;
double dmpStepSize = kukaStepWaitTime * 1e-6;
double ac = 10.0;

int main(int argc, char** args) {

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);

    // testPower();
    for(double height = 3; height < 12; height += 2) {

        for(double slope = 1; slope < 4; ++slope) {

            SegmentationTestingRewardComputer comp(height, slope, 1);
            string heightString = double_to_string(height);
            string slopeString = double_to_string(slope);
            comp.writeToFile(resolvePath("$KUKADU_HOME/movements/iros2014/artificial_part2/traj_" + heightString + "_" + slopeString + ".txt"), 1.5, 3, 0.001);

            ofstream outFile;
            outFile.open(resolvePath("$KUKADU_HOME/movements/iros2014/artificial_part2/query_" + heightString + "_" + slopeString + ".txt"));

            outFile << heightString << "\t" << slopeString << endl;
            outFile.close();

        }
    }

}

void testPower(ros::NodeHandle* node) {

    int rolloutsPerUpdate = 15;
    int importanceSamplingCount = 4;
    vector<double> rlExploreSigmas = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};

    shared_ptr<GaussianObstacleRewardComputer> reward(new GaussianObstacleRewardComputer(2, 2.0, 3));
    t_executor_res opt = reward->getOptimalTraj(5.0, 0);

    std::shared_ptr<ControlQueue> queue = std::shared_ptr<ControlQueue>(new PlottingControlQueue(1, kukaStepWaitTime));


    string execFile = "$KUKADU_HOME/movements/iros2014/2d_extended_gen/traj_0.5_3.txt";
    TrajectoryDMPLearner learner(az, bz, resolvePath(execFile));
    Dmp learnedDmp = learner.fitTrajectories();
    std::shared_ptr<DMPExecutor> learnedExec = std::shared_ptr<DMPExecutor>(new DMPExecutor(learnedDmp));

    std::vector<std::shared_ptr<Trajectory>> initTraj;
    initTraj.push_back(std::shared_ptr<Dmp>(&learnedDmp));

    PoWER pow(learnedExec, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, reward, queue, queue, ac, dmpStepSize, tolAbsErr, tolRelErr);
//    GradientDescent pow(&learnedExec, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, &reward, queue, queue, ac, dmpStepSize, tolAbsErr, tolRelErr);

    int i = 0;
    vec initT;
    vector<vec> initY;
    g1 = new Gnuplot("PoWER demo");
    while(true) {

        pow.performRollout(1, 0);
        std::shared_ptr<Dmp> lastRollout = std::dynamic_pointer_cast<Dmp>(pow.getLastUpdate());

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

    /*
    t_executor_res execRes = learnedExec.simulateTrajectory(0, 20, dmpStepSize, tolAbsErr, tolRelErr);

    g1 = new Gnuplot("PoWER demo");
    g1->set_style("lines").plot_xy(armadilloToStdVec(execRes.t), armadilloToStdVec(execRes.y[0]), "current");
    g1->set_style("lines").plot_xy(armadilloToStdVec(opt.t), armadilloToStdVec(opt.y[0]), "optimal");
    g1->showonscreen();
    */

    getchar();

}
