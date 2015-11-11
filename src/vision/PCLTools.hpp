#ifndef KUKADU_PCLTOOLS_H
#define KUKADU_PCLTOOLS_H

#include <vector>
#include <utility>
#include <boost/thread.hpp>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <boost/shared_ptr.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "../types/KukaduTypes.h"

struct FitCube {
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
    double width, height, depth;
};

class PCLTools {

private:

    bool isVisInit;
    bool keepShowingVis;

    KUKADU_SHARED_PTR<boost::thread> visThread;
    KUKADU_SHARED_PTR<pcl::visualization::PCLVisualizer> viewer;

    std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> > visPointClouds;

    void runVisThread();

public:

    PCLTools();

    static FitCube fitBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr segmentPlanar(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool negative);

    void stopVisualizationWindow();
    void visualizePointCloud(std::string id, pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

    KUKADU_SHARED_PTR<boost::thread> initializeVisualizationWindow();
    KUKADU_SHARED_PTR<pcl::visualization::PCLVisualizer> getVisualizer();

};

#endif // PCLTOOLS_H
