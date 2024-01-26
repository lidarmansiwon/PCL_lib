#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher pub;
ros::Publisher pub_downsampled;

//Filtering a PointCloud using a VoxelGrid downsampling and StatisticalOutlierRemoval filtering

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // ROS_INFO("Success to subscribe LiDAR point clouds");

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*input, *cloud);

    // VoxelGrid downsampling
    pcl::VoxelGrid<pcl::PointXYZI> sor_downsample;
    sor_downsample.setInputCloud(cloud);
    sor_downsample.setLeafSize(0.5, 0.5, 0.5); // Set the voxel size
    sor_downsample.filter(*cloud_downsampled);
    sensor_msgs::PointCloud2 downsampled;
    pcl::toROSMsg(*cloud_downsampled, downsampled);
    pub_downsampled.publish(downsampled);


    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud_downsampled << std::endl;

    // StatisticalOutlierRemoval filtering
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_filter;
    sor_filter.setInputCloud(cloud_downsampled);
    sor_filter.setMeanK(10);
    sor_filter.setStddevMulThresh(0.1);
    sor_filter.filter(*cloud_filtered);

    // Publish the Data
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    pub.publish(output);


    // 생성된 포인트클라우드 수 출력 
    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "VoxelGridAndOutlierRemoval");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/ouster/points", 1, lidar_callback, ros::TransportHints().tcpNoDelay());
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 100);
    pub_downsampled = nh.advertise<sensor_msgs::PointCloud2>("/voxelized", 100);
    // Spin
    ros::spin();
}
