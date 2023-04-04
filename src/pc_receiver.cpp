/*#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


int main (int argc, char **argv)
{
    ros::init(argc, argv, "pc_highestPoint");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("highestPoint", 1);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/zed2i/zed_node/point_cloud/cloud_registered", 1,
    [&](const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(*msg, cloud);

        float highestPoint = std::numeric_limits<float>::lowest();
        for (const auto& point : cloud)
        {
            if (point.z > highestPoint)
            {
                highestPoint = point.z;
            }
        }
        ROS_INFO("Highest Point: %f", highestPoint);

        sensor_msgs::PointCloud2 highestPoint_msg;

        pcl::PointCloud<pcl::PointXYZ> highestPoint_cloud;
        pcl::PointXYZ highestPoint_pcl;
        highestPoint_pcl.x= 0;
        highestPoint_pcl.y = 0;
        highestPoint_pcl.z = highestPoint;
        highestPoint_cloud.points.push_back(highestPoint_pcl);
        pcl::toROSMsg(highestPoint_cloud, highestPoint_msg);
        highestPoint_msg.header = msg->header;
        pub.publish(highestPoint_msg);
    });

    ros::spin ();
    return 0;
}
*/
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudProcessor
{
public:
    PointCloudProcessor() : nh_("~")
    {
        // Initialize ROS node handle
        nh_.getParam("/zed2i/zed_node/point_cloud/cloud_registered", point_cloud_topic_);
        nh_.getParam("highest_point_topic", highest_point_topic_);

        // Subscribe to point cloud topic
        point_cloud_sub_ = nh_.subscribe(point_cloud_topic_, 1, &PointCloudProcessor::pointCloudCallback, this);

        // Advertise highest point topic
        highest_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>(highest_point_topic_, 1);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        // Convert point cloud message to pcl::PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // Find highest point in point cloud
        float highest_z = -std::numeric_limits<float>::infinity();
        int highest_index = -1;
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            if (cloud->points[i].z > highest_z)
            {
                highest_z = cloud->points[i].z;
                highest_index = i;
            }
        }

        // Publish highest point
        if (highest_index != -1)
        {
            geometry_msgs::PointStamped highest_point_msg;
            highest_point_msg.header = cloud_msg->header;
            highest_point_msg.point.x = cloud->points[highest_index].x;
            highest_point_msg.point.y = cloud->points[highest_index].y;
            highest_point_msg.point.z = cloud->points[highest_index].z;
            highest_point_pub_.publish(highest_point_msg);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher highest_point_pub_;
    std::string point_cloud_topic_;
    std::string highest_point_topic_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "highest_point_node");
    PointCloudProcessor processor;
    ros::spin();
    return 0;
}
