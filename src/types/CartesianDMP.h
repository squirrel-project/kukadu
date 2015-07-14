#ifndef CARTESIANDMP_H
#define CARTESIANDMP_H

#include "DMP.h"
#include <tf/transform_datatypes.h>
#include <math.h>
#include <armadillo>


class CartesianDMP : public Dmp {
public:

    CartesianDMP(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
             double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

    CartesianDMP(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
             double tau, double az, double bz, double ax);

    CartesianDMP(std::string dmpFile);

    CartesianDMP();


    bool isCartesian();

    tf::Quaternion getQ0();
    tf::Quaternion getQg();
    tf::Quaternion getQByIdx(int idx);

    arma::vec getEta0();
    arma::vec getEtaByIdx(int idx);


    std::shared_ptr<Trajectory> copy();
};


#endif // CARTESIANDMP_H
