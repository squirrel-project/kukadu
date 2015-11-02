#ifndef PCLTOOLS_H
#define PCLTOOLS_H

#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <utility>

struct FitCube {
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
    double width, height, depth;
};

class PCLTools {

private:

    bool isVisInit;
    bool keepShowingVis;

    boost::shared_ptr<boost::thread> visThread;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> > visPointClouds;

    void runVisThread();

public:

    PCLTools();

    static FitCube fitBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr segmentPlanar(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool negative);

    void visualizePointCloud(std::string id, pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

    void stopVisualizationWindow();
    boost::shared_ptr<boost::thread> initializeVisualizationWindow();

};

#endif // PCLTOOLS_H
