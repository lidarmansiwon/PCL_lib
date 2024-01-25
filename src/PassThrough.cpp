#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

//Filtering a PointCloud using a PassThrough filter

ros::Publisher pub;

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr&input)
{
    ROS_INFO("Success to subscribe LiDAR point clouds");

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*input, *cloud);

      // 오브젝트 생성
    pcl::PassThrough<pcl::PointXYZI> cut_points;

    cut_points.setInputCloud (cloud);                //입력 
    cut_points.setFilterFieldName ("z");             //적용할 좌표 축 (eg. Z축)
    cut_points.setFilterLimits (-1.0, 1.5);          //적용할 값 (최소, 최대 값)
    cut_points.setFilterFieldName ("x");             //적용할 좌표 축 (eg. Z축)
    cut_points.setFilterLimits (-1.0, 10.0);          //적용할 값 (최소, 최대 값)
    cut_points.setFilterFieldName ("y");             //적용할 좌표 축 (eg. Z축)
    cut_points.setFilterLimits (-5.0, 5.0);          //적용할 값 (최소, 최대 값)
    //cut_points.setFilterLimitsNegative (true);     //적용할 값 외 
    cut_points.filter (*cloud_filtered);             //필터 적용 
    //Publish the Data

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    pub.publish(output);

}

int main (int argc, char** argv)
{
    //Initialize ROS
    ros::init (argc, argv, "pcl_cutting");
    ros::NodeHandle nh;

    //Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/ouster/points",1,lidar_callback, ros::TransportHints().tcpNoDelay());
    //Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cutted_points", 100);

    // Spin
    ros::spin();

}