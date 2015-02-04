#ifndef DMPUTILS
#define DMPUTILS

#include <stdio.h>
#include <iostream>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <queue>
#include <vector>
#include <cstring>
#include <armadillo>
#include <wordexp.h>
#include <memory>
#include <signal.h>
#include <utility>
#include <limits>

#include <dirent.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_poly.h>
#include <gsl/gsl_linalg.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "Tokenizer.h"
#include "../trajectory/DMPTrajectoryGenerator.h"
#include "../trajectory/TrajectoryDMPLearner.h"
#include "../learning/GaussianProcessRegressor.h"
#include "../learning/TricubeKernel.h"
#include "../learning/GaussianKernel.h"
#include "../robot/ControlQueue.h"
#include "../types/DMPBase.h"
#include "types.h"

#include "gnuplot-cpp/gnuplot_i.hpp"

int createDirectory(std::string path);

void printDoubleVector(std::vector<double>* data);
void printDoubleVector(double* data, int size);
void printDoubleMatrix(double** data, int rows, int columns);
void freeDoubleArray(double** data, int columns);
std::string resolvePath(std::string path);

int getch();

arma::mat readMovements(std::string file);
arma::mat readMovements(std::ifstream& stream);
std::pair<std::vector<std::string>, arma::mat> readSensorStorage(std::string file);

arma::vec readQuery(std::string file);
std::vector<double>* createStdVectorFromGslVector(gsl_vector* vec);

arma::vec computeDiscreteDerivatives(arma::vec x, arma::vec y);
gsl_vector* createGslVectorFromStdVector(std::vector<double>* data);
gsl_matrix* createMatrixFromQueueArray(std::queue<double>** data, int columns);
gsl_matrix* invertSquareMatrix(gsl_matrix* mat);

arma::vec createArmaVecFromDoubleArray(double* data, int n);

double* createDoubleArrayFromStdVector(std::vector<double>* data);
double* createDoubleArrayFromArmaVector(arma::vec data);
float* createFloatArrayFromStdVector(std::vector<float>* data);
double* createDoubleArrayFromVector(gsl_vector* data);
double* polyder(double* c, int len);
double* poly_eval_multiple(double* data, int data_len, double* c, int c_len);
float* copyJoints(const float* arr, int arrSize);

double** createDoubleArrayFromMatrix(gsl_matrix* data);

arma::vec stdToArmadilloVec(std::vector<double> stdVec);
arma::mat fillTrajectoryMatrix(arma::mat joints, double tMax);

std::string stringFromDouble(double d);

std::vector<double> armadilloToStdVec(arma::vec armadilloVec);
std::vector<double> computeDMPMys(std::vector<double> mys, double ax, double tau);

// taken from http://rosettacode.org/wiki/Polynomial_regression
double* polynomialfit(int obs, int degree, double *dx, double *dy);

std::vector<std::string> getFilesInDirectory(std::string folderPath);
std::vector<std::string> sortPrefix(std::vector<std::string> list, std::string prefix);

std::string buildPolynomialEquation(double* w, int paramCount);
std::vector<double>* getDoubleVectorFromArray(double* arr, int size);
std::vector<DMPBase> buildDMPBase(std::vector<double> tmpmys, std::vector<double> tmpsigmas, double ax, double tau);

arma::mat gslToArmadilloMatrix(gsl_matrix* matrix);
std::vector<double>* testGaussianRegressor();
std::vector<double> constructDmpMys(arma::mat joints);

arma::vec squareMatrixToColumn(arma::mat m);
arma::mat columnToSquareMatrix(arma::vec c);

arma::vec symmetricMatrixToColumn(arma::mat m);
arma::mat columnToSymmetricMatrix(arma::vec c);

t_executor_res executeDemo(std::shared_ptr<ControlQueue> movementQu, std::string file, double az, double bz, int plotResults);

void set_ctrlc_exit_handler();
void exit_handler(int s);

#endif
