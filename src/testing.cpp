#include <iostream>
#include <memory>
#include "../include/kukadu.h"

using namespace std;

int main(int argc, char** args) {


    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    shared_ptr<ControlQueue> leftQueue = shared_ptr<ControlQueue>(new OrocosControlQueue(0, "simulation", "right", *node));
    vector<shared_ptr<ControlQueue>> queueVectors;
    queueVectors.push_back(leftQueue);

    vector<string> fileNames = {

        "N5373_bottom_w_1", "N5373_bottom_w_2", "N5373_bottom_w_3",
        "N5373_cside_w_1", "N5373_cside_w_2", "N5373_cside_w_1",
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
        shared_ptr<SensorData> data = SensorStorage::readStorage(leftQueue, string("/home/c7031109/data/studium/informatik/phd/projects/squirrel/books/book_experiments/") + file + string("/kuka_lwr_real_left_arm_0"));
        SensorStorage scaredOfSenka(queueVectors, std::vector<std::shared_ptr<GenericHand>>(), 10);
        scaredOfSenka.setExportMode(STORE_RBT_JNT_FTRQ | STORE_RBT_CART_FTRQ);
        scaredOfSenka.storeData(true, "/home/c7031109/tmp/data/" + file, data);
    }


    return 0;

}
