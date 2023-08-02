#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>

class MechSort
{
public:
    MechSort() : nh_()
    {
    // Initialize the ROS node

    // Subscribe to the "highest_voxel" topic
    voxel_subscriber_ = nh_.subscribe("highest_voxel", 1, &MechSort::voxelCallback, this);

    // Move Group interface for the sia10f_manipulator
    move_group_ = new moveit::planning_interface::MoveGroupInterface("sia10f_manipulator");

    // Set the target end-effector link
    move_group_->setEndEffectorLink("sia10f_tool0");
    
    // Advertise the target pose publisher
    target_pose_publisher_ = nh_.advertise<geometry_msgs::Pose>("mechsort_target_pose", 1);
    yn_check_publisher_ = nh_.advertise<std_msgs::Bool>("mechsort_yn_check", 1);
    }

    void voxelCallback(const geometry_msgs::Point::ConstPtr& voxel_msg)
    {
        // Set the target pose
        geometry_msgs::Pose target_pose;
        target_pose.position.x = voxel_msg->x;
        target_pose.position.y = voxel_msg->y;
        target_pose.position.z = voxel_msg->z;
        target_pose.orientation.w = 1.0;  // Assuming you want to keep the same orientation
        
        target_pose_publisher_.publish(target_pose);

        std::cout << "Do you want to proceed with the movement? (Y/N): ";
        char yn;
        std::cin >> yn;

        if (yn == 'Y' || yn == 'y')
        {
            // Plan and execute the movement
            move_group_->setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success)
            {
                move_group_->move();
            }
            else
            {
                ROS_WARN("Failed to plan the movement.");
            }
        }
        else
        {
            ROS_INFO("Movement canceled by the user.");
        }
    }
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber voxel_subscriber_;
    moveit::planning_interface::MoveGroupInterface* move_group_;
    ros::Publisher target_pose_publisher_;
    ros::Publisher yn_check_publisher_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mechsort_node");
    // ros::NodeHandle nh;
    MechSort mechsort;
    ros::spin();
    return 0;
}
