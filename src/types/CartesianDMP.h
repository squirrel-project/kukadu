#ifndef CARTESIANDMP_H
#define CARTESIANDMP_H

#include "DMP.h"

class CartesianDMP : public Dmp {
public:

    CartesianDMP(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
             double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

    CartesianDMP(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
             double tau, double az, double bz, double ax);

    CartesianDMP(std::string dmpFile);

    CartesianDMP();


    bool isCartesian();

    std::shared_ptr<Trajectory> copy();
};


#endif // CARTESIANDMP_H
