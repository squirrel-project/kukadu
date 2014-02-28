#ifndef DMPGENERALIZER
#define DMPGENERALIZER

#include <string>
#include <vector>
#include <armadillo>

#include "../types/DictionaryTrajectory.h"
#include "../utils/types.h"
#include "../utils/utils.h"
#include "../types/DMP.h"
#include "../types/DMPBase.h"
#include "../types/QueryPoint.h"
#include "../trajectory/DMPTrajectoryGenerator.h"
#include "../trajectory/TrajectoryDMPLearner.h"
#include "../learning/GenericKernel.h"
#include "../learning/GaussianProcessRegressor.h"
#include "../learning/LWRRegressor.h"
#include "GenericGeneralizer.h"

/**
 * \defgroup ControlPolicyFramework
 * This framework provides several methods for control policy learning, execution and reinforcement
 */

/** \brief This class is able to generalize task specific trajectories from previous examples
 * 
 * This method works well on tasks, where the sample trajectories are similar in shape. Therefore, several simple trajectories
 * have to be measured and stored in files.
 * \ingroup ControlPolicyFramework
 */
class DMPGeneralizer : public GenericGeneralizer {
	
private:
	
	double ax;
	double tau;
	
	std::string baseFolder;
	DictionaryTrajectory* dictTraj;
	
	std::vector<QueryPoint> mapFiles(std::vector<std::string> queryFiles, std::vector<std::string> trajFiles, std::string prefix1, std::string prefix2);
	arma::mat computeCovarianceMatrix(int level, GenericKernel* kernel, double beta);
	
	double degOfFreedom;

public:

	/**
	 * \brief constructor
	 * \param baseFolder folder that contains the sample trajectory files
	 * \param degOfFreedom number of degrees of freedom
	 * \param tmpmys defines basis functions for dynamic movement primitives
	 * \param tmpsigmas defines basis functions for dynamic movement primitives
	 * \param az dmp az parameter
	 * \param bz dmp bz parameter
	 */
	DMPGeneralizer(std::string baseFolder, int degOfFreedom, std::vector<double> tmpmys, std::vector<double> tmpsigmas, double az, double bz, double ax, double tau);
	
	/**
	 * \brief returns number of trajectory samples
	 */
	int getQueryPointCount();
	
	double getDegOfFreedom();
	
	/**
	 * \brief returns a sample point by index
	 * \param index index of sample point
	 */
	QueryPoint getQueryPointByIndex(int index);
	
	/**
	 * \brief returns the generalized trajectory at a certain query point
	 * \param trajectoryKernel kernel for trajectory shape generalization
	 * \param parameterKernel kernel for trajectory goal generalization
	 * \param query required query point
	 * \param beta beta for Gaussian processes
	 */
	Dmp generalizeDmp(GenericKernel* trajectoryKernel, GenericKernel* parameterKernel, arma::vec query, double beta);
    
};

#endif