#ifndef GENERALDMPLEARNER_H
#define GENERALDMPLEARNER_H

#include "DMPTrajectoryGenerator.h"
#include "../utils/utils.h"
#include "../utils/types.h"
#include "../types/DMPBase.h"
#include "../learning/GeneralFitter.h"

#include <vector>
#include <queue>
#include <cstdlib>
#include <armadillo>

class GeneralDmpLearner {

private:

    int degFreedom;
    arma::mat joints;

    std::vector<DMPBase> dmpBase;

    void construct(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, arma::mat joints, int degFreedom);
    trajectory_learner_internal fitTrajectory(arma::vec time, arma::vec y, arma::vec dy, arma::vec ddy);

protected:

    double az;
    double bz;
    double ax;
    double tau;

    virtual std::shared_ptr<Dmp> createDmpInstance(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                                                   double tau, double az, double bz, double ax) = 0;

    virtual arma::vec computeFitY(arma::vec& time, arma::vec& y, arma::vec& dy, arma::vec& ddy, arma::vec& vec_g) = 0;

public:

    /**
     * \brief constructor
     * \param dmpBase dmp basis function definition
     * \param tau dmp timing constant
     * \param az dmp az constant
     * \param bz dmp bz constant
     * \param ax dmp ax constant
     * \param joints measured joints
     * \param degFreedom robots degrees of freedom
     */
    GeneralDmpLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, arma::mat joints);

    /**
     * \brief constructor
     * \param dmpBase dmp basis function definition
     * \param tau dmp timing constant
     * \param az dmp az constant
     * \param bz dmp bz constant
     * \param ax dmp ax constant
     * \param file file containing the measured joints
     * \param degFreedom robots degrees of freedom
     */
    GeneralDmpLearner(std::vector<double> mysDef, std::vector<double> sigmasDef, double az, double bz, std::string file);

    GeneralDmpLearner(double az, double bz, std::string file);

    GeneralDmpLearner(double az, double bz, arma::mat joints);

    /**
     * \brief fit the specified trajectories
     */
    std::shared_ptr<Dmp> fitTrajectories();

};

#endif // GENERALDMPLEARNER_H
