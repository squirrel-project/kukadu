#include <iostream>
#include <string>
#include <armadillo>
#include <thread>
#include <vector>
#include <boost/program_options.hpp>

#include "../../include/kukadu.h"

using namespace std;
using namespace arma;
namespace po = boost::program_options;

void switch2dQueryPoint();

bool stopThread;
double k, d, currentY;
shared_ptr<DictionaryGeneralizer> dmpGen;

int main(int argc, char** args) {

    int importanceSamplingCount;
    double tau, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ac, as, alpham;
    string inDir, cfFile;
    vector<double> rlExploreSigmas;

    // only one time center selected
    vec timeCenters(1); timeCenters(0) = 2.0;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("metric.database", po::value<string>(), "database folder")
            ("metric.goldStandard", po::value<string>(), "gold standard file")
            ("metric.alpham", po::value<double>(), "alpham")
            ("metric.exploreSigmas", po::value<string>(), "reinforcement learning exploration sigmas")
            ("metric.importanceSamplingCount", po::value<int>(), "size of importance sampling vector")
            ("dmp.tau", po::value<double>(), "tau")
            ("dmp.az", po::value<double>(), "az")
            ("dmp.bz", po::value<double>(), "bz")
            ("dmp.dmpStepSize", po::value<double>(), "dmp time step size")
            ("dmp.tolAbsErr", po::value<double>(), "tolerated absolute error")
            ("dmp.tolRelErr", po::value<double>(), "tolerated relative error")
            ("dmp.ac", po::value<double>(), "ac")
            ("dmp.as", po::value<double>(), "as")
    ;

    ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/pouring.prop"), std::ifstream::in);
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
    if (vm.count("dmp.as")) as = vm["dmp.as"].as<double>();
    else return 1;

    if (vm.count("metric.database")) inDir = resolvePath(vm["metric.database"].as<string>());
    else return 1;
    if (vm.count("metric.goldStandard")) inDir = resolvePath(vm["metric.goldStandard"].as<string>());
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

    cout << "all loaded" << endl;
    getchar();


    shared_ptr<ControlQueue> simulationQueue;
    shared_ptr<ControlQueue> executionQueue;
    shared_ptr<Gnuplot> g1;
    shared_ptr<thread> switchThr;

    int rolloutsPerUpdate = 2 * 4 * timeCenters.n_elem;

    vec trajMetricWeights(7);
    trajMetricWeights.fill(1.0);

    tau = 5.0;
    float tmp = 0.1;
    double ax = -log(tmp) / tau / tau;
    double relativeDistanceThresh = 0.6;

    vec newQueryPoint(2);
    newQueryPoint(0) = 11.0;
    newQueryPoint(1) = 234.0;

    mat m(2,2);
    m(0, 0) = 1.0;
    m(1, 0) = 0;
    m(0, 1) = 0;
    m(1, 1) = 0.0007;

    cout << "(main) creating dictionary generalizer object" << endl;
//    DictionaryGeneralizer* dmpGen = new DictionaryGeneralizer(timeCenters, newQueryPoint, queue, queue, inDir, columns - 1, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, trajMetricWeights, relativeDistanceThresh, as, alpham);
    dmpGen = std::shared_ptr<DictionaryGeneralizer>(new DictionaryGeneralizer(timeCenters, newQueryPoint, simulationQueue, executionQueue, inDir, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ax, tau, ac, as, m, relativeDistanceThresh, alpham));
    cout << "(main) done" << endl;

    std::shared_ptr<DmpRewardComputer> reward = std::shared_ptr<DmpRewardComputer>(new DmpRewardComputer(resolvePath(cfFile), az, bz, dmpStepSize, dmpGen->getDegOfFreedom(), dmpGen->getTrajectory()->getTmax()));
    cout << "execute ground truth" << endl;
    t_executor_res opt = reward->getOptimalTraj();
    cout << "execution done" << endl;

    cout << "(main) initializing trajectory" << endl;
    std::vector<std::shared_ptr<Trajectory>> initTraj;
    initTraj.push_back(dmpGen->getTrajectory());
    cout << "(main) done" << endl;

    cout << "(main) query point to maximize is: " << newQueryPoint << endl;
    std::shared_ptr<LinCombDmp> lastRollout = std::shared_ptr<LinCombDmp>(nullptr);

    cout << "(main) creating reinforcement learning environment" << endl;
    PoWER pow(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, reward, simulationQueue, executionQueue, ac, dmpStepSize, tolAbsErr, tolRelErr);
    cout << "(main) done" << endl;

    vector<std::shared_ptr<Gnuplot>> gs;
    for(int i = 0; i < dmpGen->getDegOfFreedom(); ++i)
        gs.push_back(std::shared_ptr<Gnuplot>(new Gnuplot("PoWER demo")));

    int i = 0;
    vec initT;
    vector<vec> initY;

    while( i < 2 ) {

        cout << "(main) performing rollout" << endl;
        pow.performRollout(1, 0);
        lastRollout = std::dynamic_pointer_cast<LinCombDmp>(pow.getLastUpdate());
        cout << "(main) retrieving last metric" << endl;
        vector<Mahalanobis> metrics = lastRollout->getMetric();

        cout << "used metrics: " << endl;
        for(int i = 0; i < metrics.size(); ++i)
            cout << metrics.at(i).getM() << endl;

        if(i == 0) {
            initT = pow.getLastUpdateRes().t;
            initY = pow.getLastUpdateRes().y;
        }


        if( (i % 1) == 0 ) {

            int plotNum = dmpGen->getDegOfFreedom();
            for(int plotTraj = 0; plotTraj < plotNum; ++plotTraj) {

                ostringstream convert;   // stream used for the conversion
                convert << plotTraj;

                g1 = gs.at(plotTraj);
                g1->set_style("lines").plot_xy(armadilloToStdVec(opt.t), armadilloToStdVec(opt.y[plotTraj]), "optimal trajectoy");
                g1->set_style("lines").plot_xy(armadilloToStdVec(initT), armadilloToStdVec(initY[plotTraj]), "initial trajectoy");
                g1->set_style("lines").plot_xy(armadilloToStdVec(pow.getLastUpdateRes().t), armadilloToStdVec(pow.getLastUpdateRes().y[plotTraj]), "generalized trajectory");
                g1->showonscreen();

            }

        }

        g1->reset_plot();

        if( (i % 20) == 19)
            g1->remove_tmpfiles();

        ++i;

        getchar();

    }

    cout << "(PouringExperiment) execution of trajectory at new position" << endl;

    while( 1 ) {

        cout << "(PouringExperiment) first coordinate (< 0 for exit): " << endl;
        cin >> newQueryPoint(0);

        if(newQueryPoint(0) < 0)
            break;

        cout << "(PouringExperiment) second coordinate: " << endl;
        cin >> newQueryPoint(1);

        dmpGen->setAs(as);

        lastRollout = std::dynamic_pointer_cast<LinCombDmp>(dmpGen->getTrajectory());
        lastRollout->setCurrentQueryPoint(newQueryPoint);
        dmpGen->switchQueryPoint(newQueryPoint);

        vec startingPos = dmpGen->getTrajectory()->getStartingPos();
        simulationQueue->moveJoints(startingPos);
        switchThr = shared_ptr<thread>(new std::thread(switch2dQueryPoint));
        t_executor_res updateRes = dmpGen->simulateTrajectory();
        stopThread = true;
        switchThr->join();
        simulationQueue->moveJoints(startingPos);

        char cont = 'n';
        cout << "(main) do you want to execute this trajectory? (y/N) ";
        cin >> cont;

        if(cont == 'y' || cont == 'Y') {

            cout << "(main) executing trial" << endl;

            lastRollout = std::dynamic_pointer_cast<LinCombDmp>(dmpGen->getTrajectory());
            lastRollout->setCurrentQueryPoint(newQueryPoint);
            dmpGen->switchQueryPoint(newQueryPoint);

            executionQueue->moveJoints(startingPos);
            switchThr = shared_ptr<thread>(new std::thread(switch2dQueryPoint));
            dmpGen->executeTrajectory();
            stopThread = true;
            switchThr->join();

        }

    }


}

/* switch query point for pouring */
void switch2dQueryPoint() {

    stopThread = false;

    vec qp(2);
    qp(0) = 9;
    qp(1) = 400;

    ros::Rate sleepRate(50);
    for(int i = 0; !stopThread; ++i) {

        ros::spinOnce();

        sleepRate.sleep();
        double oldQp = qp(0);
        qp(0) = k * currentY + d;

        if(oldQp != qp(0)) {
            cout << currentY << endl;
            cout << "switching execution to position " << qp(0) << endl;
            dmpGen->switchQueryPoint(qp);
        }

    }

}
