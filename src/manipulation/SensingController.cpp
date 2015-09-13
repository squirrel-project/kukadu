#include "SensingController.hpp"
#include <Python.h>
#include <boost/filesystem.hpp>

using namespace std;
namespace pf = boost::filesystem;

SensingController::SensingController(int hapticMode, string caption, std::string databasePath, vector<shared_ptr<KukieControlQueue>> queues, vector<shared_ptr<GenericHand>> hands, std::string tmpPath, std::string classifierPath, std::string classifierFile, std::string classifierFunction) : Controller(caption) {

    classifierParamsSet = false;

    this->hands = hands;
    this->queues = queues;
    this->tmpPath = tmpPath;
    this->hapticMode = hapticMode;
    this->databasePath = databasePath;
    this->classifierFile = classifierFile;
    this->classifierPath = classifierPath;
    this->classifierFunction = classifierFunction;

    bestParamC = 0.0;
    bestParamD = 0.0;
    bestParamParam1 = 0.0;
    bestParamParam2 = 0.0;

}

void SensingController::gatherData(std::string dataBasePath, std::string dataName) {

    gatherData(dataBasePath + dataName);

}

std::string SensingController::getDatabasePath() {
    return databasePath;
}

void SensingController::gatherData(std::string completePath) {

    vector<shared_ptr<ControlQueue>> castedQueues;
    for(shared_ptr<KukieControlQueue> queue : queues)
        castedQueues.push_back(queue);

    SensorStorage store(castedQueues, hands, 100);

    prepare();

    shared_ptr<thread> storageThread = store.startDataStorage(completePath);

    performCore();

    store.stopDataStorage();
    storageThread->join();

    cleanUp();

}

std::string SensingController::getFirstRobotFileName() {
    return queues.at(0)->getRobotFileName();
}

int SensingController::performClassification() {

    if(!classifierParamsSet) {
        string errorMsg = "(SensingController) classifier parameters not yet set" ;
        cerr << errorMsg << endl;
        throw errorMsg;
    }

    int classifierRes = -1;

    pf::remove_all(tmpPath + "hapticTest");

    gatherData(tmpPath, "hapticTest");

    if(hapticMode == SensingController::HAPTIC_MODE_TERMINAL) {
        cout << "what was the haptic result? [0, 3]" << endl;
        cin >> classifierRes;
    } else if(hapticMode == SensingController::HAPTIC_MODE_CLASSIFIER) {
        vector<double> res = callClassifier(databasePath, tmpPath + "hapticTest/" + queues.at(0)->getRobotFileName() + "_0", true, bestParamC, bestParamD, bestParamParam1, bestParamParam2);
        vector<double> cutAwayRes(res.begin(), res.end() - 1);
        int maxIdx = 0;
        double maxElement = cutAwayRes.at(0);
        for(int i = 1; i < cutAwayRes.size() - 1; ++i) {
            if(cutAwayRes.at(i) > maxIdx) {
                maxElement = cutAwayRes.at(i);
                maxIdx = i;
            }
        }
        classifierRes = maxIdx;

    } else {
        throw "haptic mode not known";
    }

    cout << "(main) classifier result is category " << classifierRes << endl << "(main) press enter to continue" << endl;
    getchar();

    pf::remove_all(tmpPath + "hapticTest");

    return classifierRes;

}

double SensingController::createDataBase() {

    int numClasses = 0;
    string path = getDatabasePath();
    vector<pair<int, string>> collectedSamples;
    cout << "(SensingController) data is stored to " << path << endl;
    if(!fileExists(path)) {

        cout << "(SensingController) folder doesn't exist - create" << endl;
        createDirectory(path);

        // create the database
        numClasses = getSensingCatCount();
        cout << "(SensingController) " << getCaption() << " offers " << numClasses << " classes" << endl;

        for(int currClass = 0; currClass < numClasses; ++currClass) {

            int cont = 1;
            for(int sampleNum = 0; cont == 1; ++sampleNum) {

                cout << "(SensingController) press key to collect sample number " << sampleNum << " for class " << currClass << endl;
                getchar();

                stringstream s;
                s << "class_" << currClass << "_sample_" << sampleNum;
                string relativePath = s.str();
                string relativeClassifyPath = relativePath + "/" + getFirstRobotFileName() + "_0";
                string nextSamplePath = path + relativePath;
                gatherData(nextSamplePath);

                collectedSamples.push_back(pair<int, string>(currClass, relativeClassifyPath));

                cout << "(SensingController) want to collect another sample for class " << currClass << "? (0 = no / 1 = yes): ";
                cin >> cont;

            }

        }

        writeLabelFile(path, collectedSamples);

    } else {
        cout << "(SensingController) database for controller " << getCaption() << " exists - not collection required" << endl;
    }

    // if no classifier file exists
    if(!fileExists(path + "classRes")) {

        // determine confidence value on database
        vector<double> classRes = callClassifier(path, "/home/c7031109/tmp/kuka_lwr_real_left_arm_0", false, 0.0, 0.0, 0.0, 0.0);

        double confidence = classRes.at(classRes.size() - 5);
        double bestParamC = classRes.at(classRes.size() - 4);
        double bestParamD = classRes.at(classRes.size() - 3);
        double bestParamPar1 = classRes.at(classRes.size() - 2);
        double bestParamPar2 = classRes.at(classRes.size() - 1);

        ofstream ofile;
        ofile.open(path + "classRes");
        ofile << confidence << "\t" << bestParamC << "\t" << bestParamD << "\t" << bestParamPar1 << "\t" << bestParamPar2 << endl;
        ofile.close();

    }

    ifstream infile;
    infile.open(path + "classRes");
    double confidence = 0.0;
    double bestParamC = 0.0;
    double bestParamD = 0.0;
    double bestParamPar1 = 0.0;
    double bestParamPar2 = 0.0;
    infile >> confidence >> bestParamC >> bestParamD >> bestParamPar1 >> bestParamPar2;

    setCLassifierParams(bestParamC, bestParamD, bestParamParam1, bestParamParam2);

    cout << "(SensingController) determined a confidence of " << confidence << endl;

    return confidence;

}

std::shared_ptr<ControllerResult> SensingController::performAction() {

    prepare();
    performCore();
    cleanUp();

    return nullptr;

}

void SensingController::setCLassifierParams(double bestParamC, double bestParamD, double bestParamParam1, double bestParamParam2) {
    classifierParamsSet = true;
    this->bestParamC = bestParamC;
    this->bestParamD = bestParamD;
    this->bestParamParam1 = bestParamParam1;
    this->bestParamParam2 = bestParamParam2;
}

std::vector<double> SensingController::callClassifier(std::string trainedPath, std::string passedFilePath, bool classify, double bestParamC, double bestParamD, double bestParamParam1, double bestParamParam2) {

    vector<double> retVals;
    string mName = classifierFile;
    string fName = classifierFunction;
    string argumentVal = trainedPath;

    PyObject *pName, *pModule, *pFunc;
    PyObject *pArgs, *pValue;

    Py_Initialize();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString(string(string("sys.path.append('") + classifierPath + string("')")).c_str());
    PyRun_SimpleString("import trajlab_main");

    pName = PyUnicode_FromString(mName.c_str());
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {

        pFunc = PyObject_GetAttrString(pModule, fName.c_str());

        if (pFunc && PyCallable_Check(pFunc)) {

            pArgs = PyTuple_New(7);
            pValue = PyUnicode_FromString(argumentVal.c_str());

            if (!pValue) {

                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert argument\n");
                return retVals;

            }

            PyTuple_SetItem(pArgs, 0, pValue);

            pValue = PyUnicode_FromString(passedFilePath.c_str());

            if (!pValue) {

                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert argument\n");

            }

            PyTuple_SetItem(pArgs, 1, pValue);

            pValue = PyFloat_FromDouble((classify) ? 1.0 : -1.0);

            if (!pValue) {

                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert argument\n");

            }

            PyTuple_SetItem(pArgs, 2, pValue);

            pValue = PyFloat_FromDouble(bestParamC);

            if (!pValue) {

                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert argument\n");

            }

            PyTuple_SetItem(pArgs, 3, pValue);

            pValue = PyFloat_FromDouble(bestParamD);

            if (!pValue) {

                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert argument\n");

            }

            PyTuple_SetItem(pArgs, 4, pValue);

            pValue = PyFloat_FromDouble(bestParamParam1);

            if (!pValue) {

                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert argument\n");

            }

            PyTuple_SetItem(pArgs, 5, pValue);

            pValue = PyFloat_FromDouble(bestParamParam2);

            if (!pValue) {

                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert argument\n");

            }

            PyTuple_SetItem(pArgs, 6, pValue);

            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);

            if (pValue != NULL) {

                int count = (int) PyList_Size(pValue);
                for(int i = 0; i < count; ++i) {
                    PyObject* ptemp = PyList_GetItem(pValue, i);
                    retVals.push_back(PyFloat_AsDouble(ptemp));
                }

                // retVal = PyLong_AsLong(pValue);
                Py_DECREF(pValue);

            } else {

                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr,"Call failed\n");

            }

        } else {

            if (PyErr_Occurred())
                PyErr_Print();
            cerr << "Cannot find function " << fName << endl;

        }

        Py_XDECREF(pFunc);
        Py_DECREF(pModule);

    }
    else {

        PyErr_Print();
        cerr << "Failed to load " << mName << endl;

    }

    Py_Finalize();

    return retVals;

}

void SensingController::writeLabelFile(std::string baseFolderPath, std::vector<std::pair<int, std::string>> collectedSamples) {

    ofstream outFile;
    outFile.open(baseFolderPath + "labels");

    for(pair<int, string> sample : collectedSamples)
        outFile << sample.second << " " << sample.first << endl;

}

