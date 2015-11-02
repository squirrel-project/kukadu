#ifndef PCTRANSFORMATOR_H
#define PCTRANSFORMATOR_H

#include <pcl_ros/transforms.h>

class PCTransformator {

private:


public:

    virtual pcl::PointCloud<pcl::PointXYZ> transformPc(pcl::PointCloud<pcl::PointXYZ> pc) = 0;

};



#endif // PCTRANSFORMATOR_H
