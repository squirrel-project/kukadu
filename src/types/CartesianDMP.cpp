#include "CartesianDMP.h"
#include "../utils/utils.h"

CartesianDMP::CartesianDMP(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                        double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) : Dmp(supervisedTs, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices,
                                                                                                                                              tau, az, bz, ax, ac, dmpStepSize, tolAbsErr, tolRelErr) {

}

CartesianDMP::CartesianDMP(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                   double tau, double az, double bz, double ax) : Dmp(supervisedTs, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax) {

}

CartesianDMP::CartesianDMP() {
}

bool CartesianDMP::isCartesian() {
    return true;
}

std::shared_ptr<Trajectory> CartesianDMP::copy() {

    return std::shared_ptr<Trajectory>(new CartesianDMP(*this));

}

tf::Quaternion CartesianDMP::getQ0() {
    return getQByIdx(0);
}

tf::Quaternion CartesianDMP::getQByIdx(int idx) {
    return tf::Quaternion(fitYs.at(idx)(3), fitYs.at(idx)(4), fitYs.at(idx)(5), fitYs.at(idx)(6));
}

arma::vec CartesianDMP::getEta0() {

    return getEtaByIdx(0);

}

arma::vec CartesianDMP::getEtaByIdx(int idx) {

    arma::vec omega(3);
    if(idx < getSampleCount()) {
        tf::Quaternion q0 = getQByIdx(idx);
        tf::Quaternion q1 = getQByIdx(idx + 1);

        arma::vec logL= log(q1 * q0.inverse());
        for (int i = 0; i < 3; i++)
            omega(i) = 2 * logL[i];
    } else {
        omega.fill(0.0);
    }

    return getTau() * omega;
}

tf::Quaternion CartesianDMP::getQg() {
    return getQByIdx(getSampleCount() - 1);
}
