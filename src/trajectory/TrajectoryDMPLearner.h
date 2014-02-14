#ifndef TRAJECTORYDMPLEARNER
#define TRAJECTORYDMPLEARNER

#include "../utils/utils.h"
#include "TrajectoryDMPLearner.h"
#include "PolyTrajectoryGenerator.h"
#include "DMPTrajectoryGenerator.h"
#include "../trajectory/DMPExecutor.h"
#include "../learning/GeneralFitter.h"
#include "../utils/types.h"
#include "../types/DMP.h"
#include "../types/DMPBase.h"

#include <vector>
#include <queue>
#include <cstdlib>
#include <armadillo>

/** \brief The TrajectoryDMPLearner encapsulates the dmp learning process
 * 
 * Dynamic movement primitives can be easily learned by using this class and providing the joint data or a file containing this data. Basically this is a helper
 * that enables the programmer to reduce code complexity.
 * \ingroup ControlPolicyFramework
 */
class TrajectoryDMPLearner {

private:
	
	int degFreedom;
	arma::mat joints;

	std::vector<DMPBase> dmpBase;

	double tau;
	double az;
	double bz;
	double ax;


	void construct(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, arma::mat joints, int degFreedom);
	trajectory_learner_internal fitTrajectory(arma::vec time, arma::vec y, arma::vec dy, arma::vec ddy);


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
	TrajectoryDMPLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, arma::mat joints, int degFreedom);
	
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
	TrajectoryDMPLearner(std::vector<double> mysDef, std::vector<double> sigmasDef, double az, double bz, std::string file, int degFreedom);
	
	/**
	 * \brief fit the specified trajectories
	 */
	Dmp fitTrajectories();

};


#endif