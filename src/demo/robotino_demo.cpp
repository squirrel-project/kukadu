#include <vector>
#include <iostream>
//#include <kukadu/kukadu.h>

#include <moveit/move_group_interface/move_group.h>

using namespace std;

int main(int argc, char** args) {

    /*
    RobotinoQueue robotino;
    cout << robotino.getCurrentJoints() << endl;
    */
    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    moveit::planning_interface::MoveGroup group("arm");
    group.setEndEffectorLink("link5");

    group.startStateMonitor();

    /*
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    */

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    ros::spinOnce();

    vector<double> jointVals = group.getCurrentJointValues();
    for(int i = 0; i < jointVals.size(); ++i) {
        cout << jointVals.at(i) << " ";
    }

    ros::spinOnce();

    geometry_msgs::PoseStamped currPose = group.getCurrentPose();
    cout << currPose << endl;

    geometry_msgs::Pose newPose;
    // 0 0 0 0 position
    newPose.position.x = -0.00165411;
    newPose.position.y = -0.000955;
    newPose.position.z = 0.86;
    newPose.orientation.x = 0;
    newPose.orientation.y = 0;
    newPose.orientation.z = -0.5;
    newPose.orientation.w = 0.866025;

    /*
    // some nice forward position
    currPose.pose.position.x = 0.0290346;
    currPose.pose.position.y = -0.0276921;
    currPose.pose.position.z = 0.784474;
    currPose.pose.orientation.x = 0.000497247;
    currPose.pose.orientation.y = 0.00102514;
    currPose.pose.orientation.z = -0.500843;
    currPose.pose.orientation.w = 0.865538;
    */

    jointVals.at(4) = 0.1;
    group.setJointValueTarget(jointVals);
    //group.setPoseTarget(newPose);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

    getchar();

//    group.move();


    /*
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.0;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.0;
    group.setPoseTarget(target_pose1);
    */

    getchar();

    return 0;

}
