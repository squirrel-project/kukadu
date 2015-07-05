#include "../include/kukadu.h"

#include <memory>
#include <iostream>
#include <boost/program_options.hpp>

/*
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
*/

namespace po = boost::program_options;

using namespace std;
//using namespace pcl;

/*
struct FitCube {
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
    double width, height, depth;
};
*/

/*
PointCloud<PointXYZ>::Ptr segmentPlanar(PointCloud<PointXYZ>::Ptr cloud, bool negative);
FitCube fitBox(PointCloud<PointXYZ>::Ptr cloud);
*/

int main(int argc, char** args) {

    int importanceSamplingCount;
    double tau, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ac, as, alpham;
    string inDir, cfFile, dataFolder, trajFile;
    vector<double> rlExploreSigmas;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("metric.database", po::value<string>(), "database folder")
            ("metric.goldStandard", po::value<string>(), "gold standard file")
            ("metric.alpham", po::value<double>(), "alpham")
            ("metric.exploreSigmas", po::value<string>(), "reinforcement learning exploration sigmas")
            ("metric.importanceSamplingCount", po::value<int>(), "size of importance sampling vector")
            ("metric.as", po::value<double>(), "as")
            ("dmp.tau", po::value<double>(), "tau")
            ("dmp.az", po::value<double>(), "az")
            ("dmp.bz", po::value<double>(), "bz")
            ("dmp.dmpStepSize", po::value<double>(), "dmp time step size")
            ("dmp.tolAbsErr", po::value<double>(), "tolerated absolute error")
            ("dmp.tolRelErr", po::value<double>(), "tolerated relative error")
            ("dmp.ac", po::value<double>(), "ac")
            ("mes.folder", po::value<string>(), "measurment data folder")
            ("pick.trajectory", po::value<string>(), "data for measured trajectory")
    ;

    ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/book_pick.prop"), std::ifstream::in);
    po::variables_map vm;
    po::store(po::parse_config_file(parseFile, desc), vm);
    po::notify(vm);

    if (vm.count("dmp.tau")) tau = vm["dmp.tau"].as<double>();
    else return 1;
    if (vm.count("dmp.az")) az = vm["dmp.az"].as<double>();
    else return 1;
    if (vm.count("dmp.bz")) bz = vm["dmp.bz"].as<double>();
    else return 1;
    if (vm.count("dmp.dmpStepSize")) dmpStepSize = vm["dmp.dmpStepSize"].as<double>();
    else return 1;
    if (vm.count("dmp.tolAbsErr")) tolAbsErr = vm["dmp.tolAbsErr"].as<double>();
    else return 1;
    if (vm.count("dmp.tolRelErr")) tolRelErr = vm["dmp.tolRelErr"].as<double>();
    else return 1;
    if (vm.count("dmp.ac")) ac = vm["dmp.ac"].as<double>();
    else return 1;

    if (vm.count("metric.as")) as = vm["metric.as"].as<double>();
    else return 1;
    if (vm.count("metric.database")) inDir = resolvePath(vm["metric.database"].as<string>());
    else return 1;
    if (vm.count("metric.goldStandard")) cfFile = resolvePath(vm["metric.goldStandard"].as<string>());
    else return 1;
    if (vm.count("metric.alpham")) alpham = vm["metric.alpham"].as<double>();
    else return 1;
    if (vm.count("metric.exploreSigmas")) {
        string tokens = vm["metric.exploreSigmas"].as<string>();
        stringstream parseStream(tokens);
        char bracket;
        double dToken;
        parseStream >> bracket;
        while(bracket != '}') {
            parseStream >> dToken >> bracket;
            rlExploreSigmas.push_back(dToken);
        }
    } else return 1;
    if (vm.count("metric.importanceSamplingCount")) importanceSamplingCount = vm["metric.importanceSamplingCount"].as<int>();
    else return 1;

    if (vm.count("mes.folder")) dataFolder = resolvePath(vm["mes.folder"].as<string>());
    else return 1;

    if (vm.count("pick.trajectory")) trajFile = resolvePath(vm["pick.trajectory"].as<string>());
    else return 1;

    cout << "all properties loaded" << endl;
    int kukaStepWaitTime = dmpStepSize * 1e6;

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    shared_ptr<ControlQueue> leftQueue = shared_ptr<ControlQueue>(new KukieControlQueue(kukaStepWaitTime, "real", "left_arm", *node));
    vector<shared_ptr<ControlQueue>> queueVectors;
    queueVectors.push_back(leftQueue);

    leftQueue->stopCurrentMode();

    shared_ptr<thread> lqThread = leftQueue->startQueueThread();
    leftQueue->switchMode(10);
    //leftQueue->moveJoints(stdToArmadilloVec({-0.142816, 1.02806, 1.38676, 0.855349, -0.611948, -1.11719, -1.87344}));

   // shared_ptr<SensorData> data = SensorStorage::readStorage(leftQueue, "/home/c7031109/tmp/pushing_data/kuka_lwr_real_left_arm_0");
    shared_ptr<SensorData> data = SensorStorage::readStorage(leftQueue, "/home/c7031098/testing/push_data/pushing_data/kuka_lwr_real_left_arm_0");
    data->removeDuplicateTimes();

    arma::vec times = data->getTimes();
    arma::mat jointPos = data->getJointPos();
    cout <<" (testing) get data " <<endl;
    arma::mat cartPos = data->getCartPos();
    cout <<" (testing) data loaded" <<endl;

    cout << jointPos.row(0) << endl;
    leftQueue->moveJoints(jointPos.row(0).t());

    leftQueue->stopCurrentMode();
    leftQueue->switchMode(20);

    // JointDMPLearner learner(az, bz, join_rows(times, cartPos));
    CartesianDMPLearner learner(az, bz, join_rows(times, cartPos));
    cout <<" (testing) learner created" <<endl;
    cout <<" (testing) fitting trajectories now" <<endl;
    std::shared_ptr<Dmp> leftDmp = learner.fitTrajectories();
    cout <<" (testing) trajectories fitted" <<endl;
    cout <<" (testing) creating executor" <<endl;
    DMPExecutor leftExecutor(leftDmp, leftQueue);
    cout <<" (testing) executing starting" <<endl;

    leftExecutor.executeTrajectory(ac, 0, leftDmp->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);

    /*
    int timeCount = data->getTimes().n_elem;
    ros::Rate r(75);
    arma::vec currentRow;
    for(int i = 0; i < timeCount; ++i) {
        currentRow = data->getJointPosRow(i);
        leftQueue->addJointsPosToQueue(currentRow);
        //leftQueue->synchronizeToControlQueue(1);
        r.sleep();
    }
    */


    leftQueue->stopCurrentMode();

    /*

    char* pcdFile = "/home/c7031109/tmp/pcds/with-object.pcd";

    // Initialize ROS
    ros::init(argc, args, "Bounding_Box");
    ros::NodeHandle nh;

    // Load the target cloud PCD file
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    io::loadPCDFile(pcdFile, *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size()  << " data points." << std::endl;

    cloud_filtered = segmentPlanar(cloud_filtered, true);
    cloud_filtered = segmentPlanar(cloud_filtered, true);
    cloud_filtered = segmentPlanar(cloud_filtered, true);
    cloud_filtered = segmentPlanar(cloud_filtered, false);

    FitCube cube = fitBox(cloud_filtered);

    //transform pointcloud
    //transform(*cloud_filtered, *cloud_filtered, )


    //adding the bounding box to a viewer :
    shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<PointXYZ>(cloud_filtered, "NAO arm cloud");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "NAO arm cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->addCube(cube.translation, cube.rotation, cube.width, cube.height, cube.depth);

    arma::vec pos(4);
    for(int i = 0; i < 3; ++i)
        pos(i) = cube.translation.coeff(i);
    pos(3) = 1.0;

    arma::mat kinectToWorld(4, 4);
    kinectToWorld(0, 0) = 0; kinectToWorld(0, 1) = 0.64278745651245; kinectToWorld(0, 2) = 0.76604473590851; kinectToWorld(0, 3) = -0.34577488899231;
    kinectToWorld(1, 0) = 1.0000001192093; kinectToWorld(1, 1) = -7.1525573730469e-07; kinectToWorld(1, 2) = 2.9802322387695e-07; kinectToWorld(1, 3) = 0.68909960985184;
    kinectToWorld(2, 0) = 5.9604644775391e-07; kinectToWorld(2, 1) = 0.76604473590851; kinectToWorld(2, 2) = -0.64278769493103; kinectToWorld(2, 3) = 0.65162092447281;
    kinectToWorld(3, 0) = 0.0; kinectToWorld(3, 1) = 0.0; kinectToWorld(3, 2) = 0.0; kinectToWorld(3, 3) = 1.0;
    kinectToWorld = kinectToWorld;

    arma::vec transformed = kinectToWorld * pos;
    transformed = transformed / transformed(3);
    cout << "transformed:" << endl << transformed << endl;
    cout << "ground truth pos: {0.3, 0.5, 0, 1}" << endl;

    arma::vec zAxis(4); zAxis(0) = 0; zAxis(1) = 0; zAxis(2) = 3.0; zAxis(3) = 1.0;
    arma::vec zAxisInKin = inv(kinectToWorld) * zAxis;
    zAxisInKin = zAxisInKin / zAxisInKin(3);

    arma::vec newLineEndPoint(4);
    newLineEndPoint(0) = pos(0) + zAxisInKin(0);
    newLineEndPoint(1) = pos(1) + zAxisInKin(1);
    newLineEndPoint(2) = pos(2) + zAxisInKin(2);

    PointXYZ p1, p2;
    p1.data[0] = pos(0); p1.data[1] = pos(1); p1.data[2] = pos(2);
    p2.data[0] = newLineEndPoint(0); p2.data[1] = newLineEndPoint(1); p2.data[2] = newLineEndPoint(2);
    viewer->addLine(p1, p2);

    while(!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
      }

    ros::shutdown();
    return 0;

    */

    /*
    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    shared_ptr<ControlQueue> leftQueue = shared_ptr<ControlQueue>(new OrocosControlQueue(0, "simulation", "right", *node));
    vector<shared_ptr<ControlQueue>> queueVectors;
    queueVectors.push_back(leftQueue);

    vector<string> fileNames = {

        "N5373_bottom_w_1", "N5373_bottom_w_2", "N5373_bottom_w_3",
        "N5373_cside_w_1", "N5373_cside_w_2", "N5373_cside_w_3",
        "N5373_oside_w_1", "N5373_oside_w_2", "N5373_oside_w_3",
        "N5373_top_w_1", "N5373_top_w_2", "N5373_top_w_3",

        "N5472_bottom_w_1", "N5472_bottom_w_2", "N5472_bottom_w_3",
        "N5472_cside_w_1", "N5472_cside_w_2", "N5472_cside_w_3",
        "N5472_oside_w_1", "N5472_oside_w_2", "N5472_oside_w_3",
        "N5472_top_w_1", "N5472_top_w_2", "N5472_top_w_3",

        "N17983_bottom_w_1", "N17983_bottom_w_2", "N17983_bottom_w_3",
        "N17983_cside_w_1", "N17983_cside_w_2", "N17983_cside_w_3",
        "N17983_oside_w_1", "N17983_oside_w_2", "N17983_oside_w_3",
        "N17983_top_w_1", "N17983_top_w_2", "N17983_top_w_3",

        "N5309222_bottom_w_1", "N5309222_bottom_w_2", "N5309222_bottom_w_3",
        "N5309222_cside_w_1", "N5309222_cside_w_2", "N5309222_cside_w_3",
        "N5309222_oside_w_1", "N5309222_oside_w_2", "N5309222_oside_w_3",
        "N5309222_top_w_1", "N5309222_top_w_2", "N5309222_top_w_3"

        };

    for(string file : fileNames) {
        shared_ptr<SensorData> data = SensorStorage::readStorage(leftQueue, string("/home/c7031109/data/studium/informatik/phd/projects/squirrel/books/book_experiments_raw/") + file + string("/kuka_lwr_real_left_arm_0"));
        SensorStorage scaredOfSenka(queueVectors, std::vector<std::shared_ptr<GenericHand>>(), 10);
        scaredOfSenka.setExportMode(STORE_RBT_JNT_FTRQ | STORE_RBT_CART_FTRQ | STORE_CART_ABS_FRC);
        scaredOfSenka.storeData(true, "/home/c7031109/tmp/data/" + file, data);
    }
    */


    return 0;

}

/*
// according to http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
FitCube fitBox(PointCloud<PointXYZ>::Ptr cloud) {

    FitCube retCube;
    PCA<PointXYZ> pca;
    PointCloud<PointXYZ> proj;

    pca.setInputCloud(cloud);
    pca.project(*cloud, proj);

    PointXYZ proj_min;
    PointXYZ proj_max;
    getMinMax3D(proj, proj_min, proj_max);

    PointXYZ min;
    PointXYZ max;
    pca.reconstruct(proj_min, min);
    pca.reconstruct(proj_max, max);
    std::cout << " min.x= " << min.x << " max.x= " << max.x << " min.y= " << min.y << " max.y= " << max.y << " min.z= " << min.z << " max.z= " << max.z << std::endl;

    //Rotation of PCA
    Eigen::Matrix3f rot_mat = pca.getEigenVectors();

    //translation of PCA
    Eigen::Vector3f cl_translation = pca.getMean().head(3);

    Eigen::Matrix3f affine_trans;
    std::cout << rot_mat << std::endl;
    //Reordering of principal components
    affine_trans.col(2) << (rot_mat.col(0).cross(rot_mat.col(1))).normalized();
    affine_trans.col(0) << rot_mat.col(0);
    affine_trans.col(1) << rot_mat.col(1);
    //affine_trans.col(3) << cl_translation,1;

    std::cout << affine_trans << std::endl;

    retCube.rotation = Eigen::Quaternionf(affine_trans);
    Eigen::Vector4f t = pca.getMean();

    retCube.translation = Eigen::Vector3f(t.x(), t.y(), t.z());

    retCube.width = fabs(proj_max.x-proj_min.x);
    retCube.height = fabs(proj_max.y-proj_min.y);
    retCube.depth = fabs(proj_max.z-proj_min.z);

    return retCube;

}

PointCloud<PointXYZ>::Ptr segmentPlanar(PointCloud<PointXYZ>::Ptr cloud, bool negative) {

    // Create the segmentation object for the planar model and set all the parameters
    PointCloud<PointXYZ>::Ptr cloud_f(new PointCloud<PointXYZ>);
    SACSegmentation<PointXYZ> seg;
    PointIndices::Ptr inliers(new PointIndices);
    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    PointCloud<PointXYZ>::Ptr cloud_plane(new PointCloud<PointXYZ> ());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size () == 0) {
        cerr << "Could not estimate a planar model for the given dataset." << endl;
        return cloud_f;
    }

    // Extract the planar inliers from the input cloud
    ExtractIndices<PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);
    cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative(negative);
    extract.filter(*cloud_f);

    return cloud_f;

}
*/
