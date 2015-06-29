#ifndef JOINTDMP_H
#define JOINTDMP_H

class JointDmp {
public:

    JointDmp(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
        double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

    bool CartesianDmp::isCartesian();
};

#endif // JOINTDMP_H
