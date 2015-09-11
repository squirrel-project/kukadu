#include "ComplexController.hpp"

using namespace std;

ComplexController::ComplexController(std::string caption, std::vector<std::shared_ptr<SensingController>> sensingControllers,
                                     std::vector<std::shared_ptr<Controller>> preparationControllers,
                                     std::string corrPSPath, std::shared_ptr<std::mt19937> generator, int stdReward, double gamma) : Controller(caption) {

    bool existsPs = fileExists(corrPSPath);
    shared_ptr<ProjectiveSimulator> projSim = nullptr;
    shared_ptr<ManualReward> manualRew = shared_ptr<ManualReward>(new ManualReward(generator, preparationControllers.size(), sensingControllers.size(), false, stdReward));

    if(existsPs) {
        // skill was already initialized and can be loaded again
        projSim = shared_ptr<ProjectiveSimulator>(new ProjectiveSimulator(manualRew, generator, corrPSPath));
    } else {
        // skill is used the first time; do initialization
        projSim = shared_ptr<ProjectiveSimulator>(new ProjectiveSimulator(manualRew, generator, gamma, PS_USE_ORIGINAL, false));
    }

}

std::shared_ptr<ControllerResult> ComplexController::performAction() {

}

std::vector<string> ComplexController::createSensingDatabase() {



}

std::vector<std::string> ComplexController::createSensingDatabase(std::vector<std::string> databasePaths, std::vector<std::shared_ptr<SensingController>> sensingControllers) {

    for(int i = 0; i < databasePaths.size(); ++i) {
        if(!fileExists(databasePaths.at(i)))
            databasePaths.at(i) = createDataBaseForSingleSense(databasePaths.at(i), sensingControllers.at(i));
    }

    return databasePaths;

}

double ComplexController::createDataBaseForSingleSense(std::string path, std::shared_ptr<SensingController> sensingController) {

    vector<pair<int, string>> collectedSamples;
    int numClasses = 4;
    cout << "(ComplexController) data is stored to " << path << endl;
    if(!fileExists(path)) {

        cout << "(ComplexController) folder doesn't exist - create" << endl;
        createDirectory(path);

        // create the database
        cout << "(ComplexController) how many different classes are there? [1, inf]" << endl;
        cin >> numClasses;

        for(int currClass = 0; currClass < numClasses; ++currClass) {

            int cont = 1;
            for(int sampleNum = 0; cont == 1; ++sampleNum) {

                cout << "(ComplexController) press key to collect sample number " << sampleNum << " for class " << currClass << endl;
                getchar();

                stringstream s;
                s << "class_" << currClass << "_sample_" << sampleNum;
                string relativePath = s.str();
                string relativeClassifyPath = relativePath + "/" + sensingController->getFirstRobotFileName() + "_0";
                string nextSamplePath = path + relativePath;
                sensingController->gatherData(nextSamplePath);

                collectedSamples.push_back(pair<int, string>(currClass, relativeClassifyPath));

                cout << "(ComplexController) want to collect another sample for class " << currClass << "? (0 = no / 1 = yes): ";
                cin >> cont;

            }

        }

        writeLabelFile(path, collectedSamples);

    } else {
        cout << "(ComplexController) database for controller " << sensingController->getCaption() << " exists - not collection required" << endl;
    }

    // if no classifier file exists
    if(!fileExists(path + "classRes")) {

        // determine confidence value on database
        vector<double> classRes = sensingController->callClassifier(path, "/home/c7031109/tmp/kuka_lwr_real_left_arm_0", false);
        for(double c : classRes)
            cout << c << "\t";
        cout << endl;
        double confidence = classRes.at(classRes.size() - 1);

        ofstream ofile;
        ofile.open(path + "classRes");
        ofile << confidence << endl;

    }

    ifstream infile;
    infile.open(path + "classRes");
    double confidence = 0.0;
    infile >> confidence;

    cout << "(ComplexController) determined a confidence of " << confidence << endl;

    return confidence;


}

void ComplexController::writeLabelFile(std::string baseFolderPath, std::vector<std::pair<int, std::string>> collectedSamples) {

    ofstream outFile;
    outFile.open(baseFolderPath + "labels");

    for(pair<int, string> sample : collectedSamples)
        outFile << sample.second << " " << sample.first << endl;

}
