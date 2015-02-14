#include "../utils/easyloggingpp/src/easylogging++.h"
_INITIALIZE_EASYLOGGINGPP

#include <geometry_msgs/Pose.h>
#include <iostream>
#include <string>
#include <armadillo>
#include <thread>
#include <vector>
#include <boost/program_options.hpp>

#include "../../include/kukadu.h"


#define ROBOT_TYPE "real"
#define ROBOT_SIDE "left"

#define CONTROL_RIGHT false

using namespace std;
using namespace ros;
using namespace arma;
using namespace geometry_msgs;
namespace po = boost::program_options;

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

    shared_ptr<RosSchunk> leftHand = shared_ptr<RosSchunk>(new RosSchunk(*node, ROBOT_TYPE, ROBOT_SIDE));
    vector<double> leftHandJoints = {0, -0.38705404571511043, 0.7258474179992682, 0.010410616072391092, -1.2259735578027993, -0.4303327436948519, 0.8185967300722126};
    leftHand->publishSdhJoints(leftHandJoints);

    shared_ptr<RosSchunk> rightHand = shared_ptr<RosSchunk>(new RosSchunk(*node, ROBOT_TYPE, "right"));
    //vector<double> rightHandJoints = {0, -0.5231897140548627, -0.09207113810135568, 0.865287869514727, 1.5399390781924753, -0.6258846012187079, 0.01877915351042305};
    vector<double> rightHandJoints = {0, -0.5238237461313833, 0.2120872918378427, 0.8655742259109377, 1.5389379959387146, -0.6260686922290597, 0.218843743489235877};
    rightHand->publishSdhJoints(rightHandJoints);

    shared_ptr<OrocosControlQueue> leftQueue = shared_ptr<OrocosControlQueue>(new OrocosControlQueue(argc, args, kukaStepWaitTime, ROBOT_TYPE, ROBOT_SIDE + string("_arm"), *node));
    shared_ptr<thread> lqThread = leftQueue->startQueueThread();

    shared_ptr<OrocosControlQueue> rightQueue = nullptr;
    shared_ptr<thread> rqThread = nullptr;

    if(CONTROL_RIGHT) {

        rightQueue = shared_ptr<OrocosControlQueue>(new OrocosControlQueue(argc, args, kukaStepWaitTime, ROBOT_TYPE, "right_arm", *node));
        rqThread = rightQueue->startQueueThread();

    }

    usleep(1e6);

    if(CONTROL_RIGHT) {

        rightQueue->switchMode(OrocosControlQueue::KUKA_JNT_IMP_MODE);
        rightQueue->moveJoints(stdToArmadilloVec({-2.0805978775024414, 1.4412447214126587, -2.203258752822876, 1.7961543798446655, 2.526369333267212, -1.6385622024536133, -0.7103539705276489}));

    }

    // measured joints were -0.2252   1.3174  -2.1671   0.4912   0.8510  -1.5699   1.0577
    mes_result currentJoints = leftQueue->getCurrentJoints();
    cout << currentJoints.joints.t() << endl;

    /*
    cout << "(main) place book on expected position and press enter" << endl;
    getchar();
    */

//    leftQueue->setStiffness(0.2, 0.01, 0.2, 15000, 150, 2.0);
    leftQueue->switchMode(OrocosControlQueue::KUKA_STOP_MODE);
    leftQueue->switchMode(OrocosControlQueue::KUKA_JNT_IMP_MODE);
    //leftQueue->moveJoints(stdToArmadilloVec({-0.514099, 1.83194, 1.95971, -0.99676, -0.0903862, 0.987185, 1.16542}));

    /*
    leftQueue->moveJoints(stdToArmadilloVec({-0.3990752398967743, 1.7550331354141235, 1.9612526893615723, -0.6003820300102234, -0.05098132789134979, 1.200370192527771, 1.168589472770691}));

    cout << "(main) press key to continue" << endl;
    getchar();

    SensorData dat(trajFile);
    TrajectoryDMPLearner learner(az, bz, dat.getRange(0, 8));
    Dmp leftDmp = learner.fitTrajectories();
    DMPExecutor leftExecutor(leftDmp, leftQueue);
    leftExecutor.executeTrajectory(ac, 0, leftDmp.getTmax(), dmpStepSize, tolAbsErr, tolRelErr);


    // first finger
    cout << "(main) ready to move first finger?" << endl;
    leftHandJoints = {0, -0.38705404571511043, 0.7258474179992682, -0.2, -0.9, -0.8303327436948519, 0.8185967300722126};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, -0.8303327436948519, 0};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, -0.3303327436948519, 0};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, -0.3303327436948519, 1.285967300722126};
    leftHand->publishSdhJoints(leftHandJoints);

    cout << "(main) ready to move second finger?" << endl;
    // second finger follows
    leftHandJoints = {0, -0.9303327436948519, 0.8185967300722126, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, -0.8303327436948519, 0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, 0, 0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, 0, 0.5, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
    leftHand->publishSdhJoints(leftHandJoints);

    // first finger again
    leftHandJoints = {0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, -0.9303327436948519, 0.8185967300722126};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, -0.8303327436948519, 0};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, 0, 0};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, 0, 0.5};
    leftHand->publishSdhJoints(leftHandJoints);

    */

    cout << "(main) preparing to drop (remove this for book peeling again)" << endl;
    getchar();
    leftQueue->moveJoints(stdToArmadilloVec({-1.0587048530578613, 1.879784107208252, 2.1199851036071777, -1.9964550733566284, -1.975645899772644, 1.5456539392471313, 2.5950534343719482}));

    cout << "(main) finally grasp it?";
    getchar();
    leftHandJoints = {0, 0, 0.5, -0.2, -0.9, 0, 0.5};
    leftHand->publishSdhJoints(leftHandJoints);

    /*
    cout << "(main) press a key to lift it" << endl;
    getchar();
    leftQueue->moveJoints(stdToArmadilloVec({-0.9291561841964722, 1.9066647291183472, 1.9648972749710083, -0.949062168598175, -0.10840536653995514, 1.199838638305664, 1.1655352115631104}));
*/

    cout << "(main) dropping book" << endl;
    leftQueue->moveJoints(stdToArmadilloVec({-0.3186831772327423, 1.877084493637085, 2.4300241470336914, -1.6613903045654297, -2.41784930229187, 1.776635766029358, 2.6922547817230225}));

    cout << "(main) opening fingers" << endl;
    vector<double> openJoints = leftHandJoints = {0, -0.48705404571511043, 0.7258474179992682, -0.650410616072391092, 0.5, -0.5303327436948519, 0.8185967300722126};
    leftHand->publishSdhJoints(leftHandJoints);

    getchar();

    leftQueue->switchMode(OrocosControlQueue::KUKA_STOP_MODE);

    cout << "(main) starting fine grained placement" << endl;
    getchar();
    leftQueue->switchMode(OrocosControlQueue::KUKA_CART_IMP_MODE);

    cout << "(main) start moving" << endl;

    int movementDuration = 70;
    double frcVal1, frcVal2, dist1, dist2;
    mes_result mes;
    Pose currentPose = leftQueue->getCartesianPoseRf();
    Pose relativePose;
    relativePose.position.x = 0.0009; relativePose.position.y = relativePose.position.z = 0.0;
    Rate slRate(20);
    Rate waitRate(0.5);
    while(1) {

        // check back side
        mes = leftQueue->getCurrentCartesianFrcTrq();
        frcVal1 = mes.joints(4);
        cout << "1: " << mes.joints.t() << endl;
        leftHandJoints = {SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, 0.3, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
        leftHand->publishSdhJoints(leftHandJoints);
        waitRate.sleep();
        mes = leftQueue->getCurrentCartesianFrcTrq();
        frcVal2 = mes.joints(4);
        cout << "1: " << mes.joints.t() << endl;
        dist1 = abs(frcVal1 - frcVal2);
        leftHandJoints = openJoints;
        leftHand->publishSdhJoints(leftHandJoints);


        // check front side
        mes = leftQueue->getCurrentCartesianFrcTrq();
        frcVal1 = mes.joints(4);
        cout << "2: " << mes.joints.t() << endl;
        leftHandJoints = {SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, 0.6};
        leftHand->publishSdhJoints(leftHandJoints);
        waitRate.sleep();
        mes = leftQueue->getCurrentCartesianFrcTrq();
        frcVal2 = mes.joints(4);
        cout << "2: " << mes.joints.t() << endl;
        dist2 = abs(frcVal1 - frcVal2);

        leftHandJoints = openJoints;
        leftHand->publishSdhJoints(leftHandJoints);

        if(dist1 < 0.05 && dist2 < 0.05) {
            cout << "book stable with dists " << dist1 << " " << dist2 << endl;
            leftHand->setGrasp(eGID_PARALLEL);
            leftHand->closeHand(0.0, 20);
            break;
        } else if(dist1 > dist2) {
            cout << "forwards with dists " << dist1 << " " << dist2 << endl;
            relativePose.position.x = +0.0009;
            for(int i = 0; i < movementDuration; ++i) {
                currentPose = leftQueue->moveCartesianRelativeWf(currentPose, relativePose);
                slRate.sleep();
            }
        } else if(dist1 < dist2) {
            cout << "backwards with dists " << dist1 << " " << dist2 << endl;
            relativePose.position.x = -0.0009;
            for(int i = 0; i < movementDuration; ++i) {
                currentPose = leftQueue->moveCartesianRelativeWf(currentPose, relativePose);
                slRate.sleep();
            }
        }

    //    getchar();

    }

    leftQueue->switchMode(OrocosControlQueue::KUKA_STOP_MODE);
    cout << "done" << endl;
    getchar();
    getchar();


    /*******************placing book*************************/

    // preparing joint: -1.0587048530578613, 1.879784107208252, 2.1199851036071777, -1.9964550733566284, -1.975645899772644, 1.5456539392471313, 2.5950534343719482
    // preparing cartesian pose_dir_wf: 0.05717991894543589, 0.8644983475584886, 0.4805854570793575, 0.21734215181578964, -0.9751957397510774, -0.04188762248520789, -1.5343423139861556
    /* preparing cartesian pose_quat_wf:
            position:
              x: 0.0571799189454
              y: 0.864498347558
              z: 0.480585457079
            orientation:
              x: 0.569213558645
              y: -0.443778916198
              z: 0.534112599132
              w: 0.440204148465
        preparing cartesian pose_quat_rf:
            position:
              x: 0.0571799189454
              y: 0.864498347558
              z: 0.480585457079
            orientation:
              x: 0.569213558645
              y: -0.443778916198
              z: 0.534112599132
              w: 0.440204148465
    */



    // pressing down joint: -0.2579503059387207, 1.8491992950439453, 2.409682512283325, -1.6310813426971436, -2.3986194133758545, 1.8296722173690796, 2.7402396202087402
    // pressing down cartesian pose_dir_wf: [0.07881880131491087, 0.8291342215107211, 0.22191931766732076, 0.2062804784406847, -0.9781121794204191, -0.027272939377186667, -1.5688184012210034
    /* pressing down cartesian pose_quat_wf:
            position:
              x: 0.0788188013149
              y: 0.829134221511
              z: 0.221919317667
            orientation:
              x: 0.555722913663
              y: -0.452557743135
              z: 0.542491578435
              w: 0.438253522249
       pressing down cartesian pose_quat_rf:
            position:
              x: -0.540514051914
              y: -0.154238834977
              z: 0.424770444632
            orientation:
              x: -0.766787623456
              y: -0.545844270684
              z: 0.32225782789
              w: 0.101196749218

    /*

    for(int i = 0; i < leftDmp.getDegreesOfFreedom(); ++i) {
        Gnuplot g1;
        g1.plot_xy(armadilloToStdVec(res.t), armadilloToStdVec(res.y.at(i)));
        g1.plot_xy(armadilloToStdVec(dat.getTime()), armadilloToStdVec(dat.getDataByIdx(i + 1)));
        getchar();
    }

    */

    /*

    vector<shared_ptr<ControlQueue>> queues = {leftQueue};
    vector<shared_ptr<GenericHand>> hands = {leftHand};
    SensorStorage store(queues, hands, 100);
    shared_ptr<thread> storageThread = store.startDataStorage(dataFolder);

    if(storageThread)
        storageThread->join();

    */



    leftQueue->setFinish();

    if(CONTROL_RIGHT)
        rightQueue->setFinish();

    lqThread->join();
    rqThread->join();

    return EXIT_SUCCESS;

}
