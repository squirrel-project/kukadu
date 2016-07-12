#ifndef KUKADU_GAUSSIANPROCESSREGRESSOR_H
#define KUKADU_GAUSSIANPROCESSREGRESSOR_H

#include <armadillo>
#include <vector>

#include <kukadu/learning/regression/generickernel.hpp>
#include <kukadu/utils/utils.hpp>
#include <kukadu/learning/regression/kernelregressor.hpp>

namespace kukadu {

    /** \brief Implements the Gaussian process regression method
     *
     * This class inherits from the KernelRegressor and implements Gaussian process regression.
     * \ingroup LearningFramework
     */
    class GaussianProcessRegressor : public KernelRegressor {
    private:

        std::vector<arma::vec> sampleXs;
        arma::vec sampleTs;
        arma::mat coVarMatrix;

        GenericKernel* kernel;

        arma::mat computeCovarianceMatrix(GenericKernel* kernel, double beta);
        double computeKernelNormValue();

    public:

        /**
         * \brief constructor, setting all the sample data, the selected kernel and the variance beta
         * \param sampleXs vector of samples (x-axis)
         * \param sampleTs vector of samples (y-axis)
         * \param kernel the selected kernel implementation
         * \param beta the assumed sample variance
         */
        GaussianProcessRegressor(std::vector<arma::vec> sampleXs, arma::vec sampleTs, GenericKernel* kernel, double beta);

        arma::vec fitAtPosition(arma::vec pos);

    };

}

#endif
