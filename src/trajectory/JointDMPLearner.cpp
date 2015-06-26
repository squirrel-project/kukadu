#include "JointDMPLearner.h"
#include "../types/jointdmp.h"

using namespace std;
using namespace arma;

JointDMPLearner::JointDMPLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, arma::mat joints) : GeneralDmpLearner(dmpBase, tau, az, bz, ax, joints) {

}

JointDMPLearner::JointDMPLearner(std::vector<double> mysDef, std::vector<double> sigmasDef, double az, double bz, std::string file) : GeneralDmpLearner(mysDef, sigmasDef, az, bz, file) {

}

JointDMPLearner::JointDMPLearner(double az, double bz, std::string file) : GeneralDmpLearner(az, bz, file) {

}

JointDMPLearner::JointDMPLearner(double az, double bz, arma::mat joints) : GeneralDmpLearner(az, bz, joints) {

}

std::shared_ptr<Dmp> JointDMPLearner::createDmpInstance(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                                       double tau, double az, double bz, double ax) {

    return shared_ptr<Dmp>(new JointDmp(supervisedTs, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax));

}

arma::vec JointDMPLearner::computeFitY(arma::vec& time, arma::vec& y, arma::vec& dy, arma::vec& ddy, arma::vec& vec_g) {
    return tau * tau * ddy - az * (bz * (vec_g - y) - tau * dy);
}
