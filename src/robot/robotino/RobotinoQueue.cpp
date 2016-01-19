#include "RobotinoQueue.h"

namespace kukadu {

    void RobotinoQueue::run() {

    }

    void RobotinoQueue::setFinish() {

    }

    void RobotinoQueue::safelyDestroy() {

    }

    void RobotinoQueue::setInitValues() {

    }

    void RobotinoQueue::stopCurrentMode() {

    }

    void RobotinoQueue::switchMode(int mode) {

    }

    void RobotinoQueue::moveJoints(arma::vec joints) {

    }

    void RobotinoQueue::moveJointsNb(arma::vec joints) {

    }

    void RobotinoQueue::setJntPtpThresh(double thresh) {

    }

    void RobotinoQueue::setStartingJoints(arma::vec joints) {

    }

    void RobotinoQueue::addJointsPosToQueue(arma::vec joints) {

    }

    void RobotinoQueue::moveCartesian(geometry_msgs::Pose pos) {

    }

    void RobotinoQueue::moveCartesianNb(geometry_msgs::Pose pos) {

    }

    void RobotinoQueue::addCartesianPosToQueue(geometry_msgs::Pose pose) {

    }

    void RobotinoQueue::setAdditionalLoad(float loadMass, float loadPos) {

    }

    void RobotinoQueue::synchronizeToControlQueue(int maxNumJointsInQueue) {

    }

    void RobotinoQueue::setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) {

    }

    void RobotinoQueue::shutUp() {

    }

    void RobotinoQueue::startTalking() {

    }

    void RobotinoQueue::rollBack(double time) {

    }

    void RobotinoQueue::stopJointRollBackMode() {

    }

    void RobotinoQueue::startJointRollBackMode(double possibleTime) {

    }

    bool RobotinoQueue::isInitialized() {

    }

    double RobotinoQueue::getTimeStep() {

    }

    std::string RobotinoQueue::getRobotName() {

    }

    std::string RobotinoQueue::getRobotFileName() {

    }

    std::string RobotinoQueue::getRobotSidePrefix() {

    }

    std::string RobotinoQueue::getRobotDeviceType() {

    }

    std::vector<std::string> RobotinoQueue::getJointNames() {

    }

    mes_result RobotinoQueue::getCartesianPos() {

    }

    mes_result RobotinoQueue::getCurrentJoints() {

    }

    mes_result RobotinoQueue::getCurrentJntFrcTrq() {

    }

    mes_result RobotinoQueue::getCurrentCartesianFrcTrq() {

    }

    geometry_msgs::Pose RobotinoQueue::getCartesianPose() {

    }

    geometry_msgs::Pose RobotinoQueue::getCartesianPoseRf() {

    }

    geometry_msgs::Pose RobotinoQueue::moveCartesianRelativeWf(geometry_msgs::Pose basePoseRf, geometry_msgs::Pose offset) {

    }

    arma::vec RobotinoQueue::getFrcTrqCart() {

    }

    arma::vec RobotinoQueue::getStartingJoints() {

    }

    arma::vec RobotinoQueue::retrieveJointsFromRobot() {

    }

}
