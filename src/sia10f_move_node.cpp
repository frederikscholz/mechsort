#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sia10f_move_node");
    ros::NodeHandle nh;

    // Create a MoveGroupInterface object for the "sia10f_manipulator" planning group
    moveit::planning_interface::MoveGroupInterface move_group("sia10f_manipulator");
    move_group.setPlanningTime(15.0);
    std::vector<std::string> joint_names = move_group.getJoints();
    std::cout << "Joint Names: ";
    for (const auto& joint_name : joint_names) {
        std::cout << joint_name << " ";
    }
    std::cout << std::endl;

    
    // Set the joint goal tolerance
    //move_group.setGoalJointTolerance(0.01);

    // Get the current joint values
    //std::vector<double> joint_values;
    //move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), joint_values);
    std::vector<double> joint_values = move_group.getCurrentJointValues();
    std::cout << "Current Joint Values: ";
    for (size_t i = 0; i < joint_values.size(); ++i) {
        std::cout << joint_values[i] << " ";
    }
    std::cout << std::endl;
    // Modify the joint value for "sia10f_joint_7_t" by adding 30 degrees
    joint_values[6] = 0.5;  // 30 degrees in radians

    // Set the modified joint values as the target
    move_group.setJointValueTarget(joint_values);
    std::vector<double> current_joint_values = move_group.getCurrentJointValues();
    std::cout << "Joint Value Target: ";
    for (size_t i = 0; i < current_joint_values.size(); ++i) {
        std::cout << current_joint_values[i] << " ";
    }
    std::cout << std::endl;
    // Plan and execute the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    //bool success = move_group.plan(plan);
    moveit::core::MoveItErrorCode success = move_group.plan(plan);

    if (success)
    {
        move_group.execute(plan);
        ROS_INFO("Joint movement executed successfully.");
    }
    else
    {
        ROS_ERROR("Failed to plan joint movement.");
    }

    return 0;
}
