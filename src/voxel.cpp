#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>



ros::Publisher pub;

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr&input)
{
    ROS_INFO("Success to subscribe LiDAR point clouds");

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*input, *cloud);


    // make object
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter; 
    voxel_filter.setInputCloud(cloud);              // 입력
    voxel_filter.setLeafSize(0.10f, 0.10f, 0.10f);  // leaf size 10cm
    voxel_filter.filter(*cloud_filtered);           // 출력

    //Publish the Data

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    pub.publish(output);
}

int main (int argc, char** argv)
{
    //Initialize ROS
    ros::init (argc, argv, "pcl_voxelization");
    ros::NodeHandle nh;

    //Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/ouster/points",1,lidar_callback, ros::TransportHints().tcpNoDelay());
    //Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 100);

    // Spin
    ros::spin();

}