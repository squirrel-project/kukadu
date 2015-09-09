#include "SensingController.hpp"
#include <Python.h>
#include <boost/filesystem.hpp>

using namespace std;
namespace pf = boost::filesystem;

SensingController::SensingController(vector<shared_ptr<KukieControlQueue>> queues, vector<shared_ptr<GenericHand>> hands, std::string tmpPath, std::string classifierPath, std::string classifierFile, std::string classifierFunction) {
    this->hands = hands;
    this->queues = queues;
    this->tmpPath = tmpPath;
}

int SensingController::performClassification(int hapticMode, std::string databasePath) {

    int classifierRes = -1;

    pf::remove_all(tmpPath + "hapticTest");

    vector<shared_ptr<ControlQueue>> castedQueues;
    for(shared_ptr<KukieControlQueue> queue : queues)
        castedQueues.push_back(queue);

    SensorStorage store(castedQueues, hands, 100);

    prepare();
    shared_ptr<thread> storageThread = store.startDataStorage(tmpPath + "hapticTest");

    performCore();

    store.stopDataStorage();
    storageThread->join();

    if(hapticMode == SensingController::HAPTIC_MODE_TERMINAL) {
        cout << "what was the haptic result? [0, 3]" << endl;
        cin >> classifierRes;
    } else if(hapticMode == SensingController::HAPTIC_MODE_CLASSIFIER) {
        classifierRes = callClassifier(databasePath, tmpPath + "hapticTest/kuka_lwr_" + queues.at(0)->getRobotDeviceType() + "_" + queues.at(0)->getRobotSidePrefix() + "_arm_0");
    } else {
        throw "haptic mode not known";
    }

    cleanUp();

    cout << "(main) classifier result is category " << classifierRes << endl << "(main) press enter to continue" << endl;
    getchar();

    pf::remove_all(tmpPath + "hapticTest");

    return classifierRes;


}

int SensingController::callClassifier(std::string trainedPath, std::string passedFilePath) {

    int retVal = -1;
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

            pArgs = PyTuple_New(2);
            pValue = PyUnicode_FromString(argumentVal.c_str());
            if (!pValue) {
                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert argument\n");
                return 1;
            }
            PyTuple_SetItem(pArgs, 0, pValue);

            pValue = PyUnicode_FromString(passedFilePath.c_str());
            if (!pValue) {
                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert argument\n");
                return 1;
            }
            PyTuple_SetItem(pArgs, 1, pValue);

            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);
            if (pValue != NULL) {
                retVal = PyLong_AsLong(pValue);
                Py_DECREF(pValue);
            }
            else {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
                retVal = -1;
            }

        }
        else {
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
        retVal = -1;
    }
    Py_Finalize();
    return retVal;

}
