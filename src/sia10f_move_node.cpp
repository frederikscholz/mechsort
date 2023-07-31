#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

using namespace std;



int main(int argc, char** argv)
{
    ros::init(argc, argv, "sia10f_move_node");
    ros::NodeHandle nh;

    

    double pi = acos(-1);

    static const std::string PLANNING_GROUP = "sia10f_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    move_group.setPlanningTime(10.0);
    move_group.setPoseReferenceFrame("base_link");
    move_group.setGoalJointTolerance(0.01);
    move_group.setMaxAccelerationScalingFactor(0.2);
    move_group.setMaxVelocityScalingFactor(0.2);
    std::vector<double> pose3 = {101 * pi / 180, -41 * pi / 180, -8 * pi / 180, -77 * pi / 180, -28 * pi / 180, 94 * pi / 180, -33 * pi / 180};
    move_group.setJointValueTarget(pose3);
    move_group.move();
    sleep(1);

    ROS_INFO("Ready to execute pose");
    ros::AsyncSpinner spinner(5);
    spinner.start();
    ros::waitForShutdown();

    return 0;

    // // Create a MoveGroupInterface object for the "sia10f_manipulator" planning group
    // moveit::planning_interface::MoveGroupInterface move_group("sia10f_manipulator");
    // move_group.setPlanningTime(15.0);
    // std::vector<std::string> joint_names = move_group.getJoints();
    // std::cout << "Joint Names: ";
    // for (const auto& joint_name : joint_names) {
    //     std::cout << joint_name << " ";
    // }
    // std::cout << std::endl;
    
    // Set the joint goal tolerance
    //move_group.setGoalJointTolerance(0.01);

    // Get the current joint values
    //std::vector<double> joint_values;
    //move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), joint_values);
    // std::vector<double> joint_values = move_group.getCurrentJointValues();
    // std::cout << "Current Joint Values: ";
    // for (size_t i = 0; i < joint_values.size(); ++i) {
    //     std::cout << joint_values[i] << " ";
    // }static const std::string PLANNING_GROUP = "sia10f_manipulator";
    
    // std::cout << std::endl;
    // // Modify the joint value for "sia10f_joint_7_t" by adding 30 degrees
    // joint_values[6] = 0.5;  // 30 degrees in radians

    // // Set the modified joint values as the target
    // move_group.setJointValueTarget(joint_values);
    // std::vector<double> current_joint_values = move_group.getCurrentJointValues();
    // std::cout << "Joint Value Target: ";
    // for (size_t i = 0; i < current_joint_values.size(); ++i) {
    //     std::cout << current_joint_values[i] << " ";
    // }
    // std::cout << std::endl;
    // // Plan and execute the trajectory
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // //bool success = move_group.plan(plan);
    // moveit::core::MoveItErrorCode success = move_group.plan(plan);

    // if (success)
    // {
    //     move_group.execute(plan);
    //     ROS_INFO("Joint movement executed successfully.");
    // }
    // else
    // {
    //     ROS_ERROR("Failed to plan joint movement.");
    // }

    //return 0;
}
