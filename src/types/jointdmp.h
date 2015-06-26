#ifndef JOINTDMP_H
#define JOINTDMP_H

#include "DMP.h"

class JointDmp : public Dmp {
public:

    JointDmp(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
             double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

    JointDmp(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
             double tau, double az, double bz, double ax);

    JointDmp(std::string dmpFile);

    JointDmp();

    bool isCartesian();
    virtual std::shared_ptr<Trajectory> copy();

};

#endif // CARTESIANDMP_H
