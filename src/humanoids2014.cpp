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

#include <RedundantKin.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include "../include/kukadu.h"
#include "../src/utils/gnuplot-cpp/gnuplot_i.hpp"

#define DOSIMULATION 1

using namespace std;
using namespace arma;

void testPerformance(OrocosControlQueue* queue);

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

int main(int argc, char** args) {

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    OrocosControlQueue* queue = new OrocosControlQueue(argc, args, kukaStepWaitTime, "simulation", "right_arm", *node);
    queue->startQueueThread();

    testPerformance(queue);
    getch();

	return 0;

}

void testPerformance(OrocosControlQueue* queue) {

    t_executor_res demoRes = executeDemo(queue,
                               "/home/c7031109/iis_robot_sw/iis_catkin_ws/src/kukadu/movements/iros2014/real_robot_2d_pickup/traj_10-5.txt",
                               0, az, bz, 1);

}
