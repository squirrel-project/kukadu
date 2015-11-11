#ifndef PLANARCUTTRANSFORMATOR
#define PLANARCUTTRANSFORMATOR

#include <armadillo>

#include "PCTransformator.hpp"

class PlanarCutTransformator {

private:

    arma::vec normalVec;
    arma::vec plainOriginVec;

public:

    PlanarCutTransformator(arma::vec normalVec, arma::vec plainOriginVec);
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

};



#endif // PLANARCUTTRANSFORMATOR
