#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <pose_msg/pose.h>
#include <cmath>
#include <moveit/robot_state/robot_state.h>
#include <iostream>

using namespace std;

double pi = acos(-1);

bool poseCallback(pose_msg::pose::Request  &req,
                  pose_msg::pose::Response &res)

{ 
  //static const string PLANNING_GROUP = "sia10f_manipulator";
  //moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  //const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  //move_group.setPoseReferenceFrame("base_link");
  //move_group.setGoalJointTolerance(0.01);
  //move_group.setMaxAccelerationScalingFactor(0.2);
  //move_group.setMaxVelocityScalingFactor(0.2);
  //char joint_names[7] = {"panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"};
  //robot_state::JointStateGroup::getJointState joint_state(joint_names);
  
  
  ROS_INFO("robot will go to the specific pose with number=%ld", (long int)req.pose);
  
  
  
  //res.result = "successfully";
   
  if( req.pose == 1 )
  {   

      static const std::string PLANNING_GROUP = "sia10f_manipulator";
      moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
      const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      move_group.setPoseReferenceFrame("base_link");
      move_group.setGoalJointTolerance(0.01);
      move_group.setMaxAccelerationScalingFactor(0.2);
      move_group.setMaxVelocityScalingFactor(0.2);
      std::vector<double> pose1 = {105*pi/180,90*pi/180,-90*pi/180,76*pi/180,-90*pi/180,90*pi/180,3*pi/180};
      move_group.setJointValueTarget(pose1);
      move_group.move();
      sleep(1);
      

    
  }
  else if ( req.pose == 2)
  {   
      static const std::string PLANNING_GROUP = "sia10f_manipulator";
      moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
      const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      move_group.setPoseReferenceFrame("base_link");
      move_group.setGoalJointTolerance(0.01);
      move_group.setMaxAccelerationScalingFactor(0.2);
      move_group.setMaxVelocityScalingFactor(0.2);
      std::vector<double> pose2 = {98*pi/180,60*pi/180,-2*pi/180,62*pi/180,96*pi/180,87*pi/180,4*pi/180};
      move_group.setJointValueTarget(pose2);
      move_group.move();
      sleep(1);
  }
  else if ( req.pose == 3)
  {   
      static const std::string PLANNING_GROUP = "sia10f_manipulator";
      moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
      const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      move_group.setPoseReferenceFrame("base_link");
      move_group.setGoalJointTolerance(0.01);
      move_group.setMaxAccelerationScalingFactor(0.2);
      move_group.setMaxVelocityScalingFactor(0.2);
      std::vector<double> pose3 = {101*pi/180,-41*pi/180,-8*pi/180,-77*pi/180,-28*pi/180,94*pi/180,-33*pi/180};
      move_group.setJointValueTarget(pose3);
      move_group.move();
      sleep(1);

  }
  else if ( req.pose == 4)
  { static const std::string PLANNING_GROUP = "sia20f_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    move_group.setPoseReferenceFrame("base_link");
    move_group.setGoalJointTolerance(0.01);
    move_group.setMaxAccelerationScalingFactor(0.05);
    move_group.setMaxVelocityScalingFactor(0.05);
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = pi/6 + joint_group_positions[0] ;
    move_group.setJointValueTarget(joint_group_positions);
    move_group.move();

  }

  else if ( req.pose == 5)
  {   
      static const std::string PLANNING_GROUP = "sia20f_manipulator";
      moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
      //const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      move_group.setPoseReferenceFrame("base_link");
      move_group.setGoalJointTolerance(0.01);
      move_group.setMaxAccelerationScalingFactor(0.05);
      move_group.setMaxVelocityScalingFactor(0.05);
      std::vector<double> pose5 = {-1.8476288318634033,-1.2808910608291626,1.3538914918899536,-1.4726046323776245,1.8809337615966797,-1.3100498914718628,-0.5311183333396912};
      move_group.setJointValueTarget(pose5);
      move_group.move();
      sleep(1);
  }
  else if ( req.pose == 6)
  {   
      static const std::string PLANNING_GROUP = "sia20f_manipulator";
      moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
      //const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      move_group.setPoseReferenceFrame("base_link");
      move_group.setGoalJointTolerance(0.01);
      move_group.setMaxAccelerationScalingFactor(0.05);
      move_group.setMaxVelocityScalingFactor(0.05);
      std::vector<double> pose6 = {-0.97124,-1.2759,0.8694,-2.0184,2.2308,-1.0956,-0.3972};
      move_group.setJointValueTarget(pose6);
      move_group.move();
      sleep(1);
  }
  
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("pose", poseCallback);
  //cout << "Ready to receive the pose'number" << endl;
  ROS_INFO("Ready to receive the pose'number");
  ros::AsyncSpinner spinner(5);
  spinner.start();
  ros::waitForShutdown();
 
  return 0;
}

