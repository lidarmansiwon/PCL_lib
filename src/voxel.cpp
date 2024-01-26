#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

class VoxelGridAndOutlierRemovalNode : public rclcpp::Node
{
public:
    VoxelGridAndOutlierRemovalNode()
        : Node("VoxelGridAndOutlierRemoval")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); // 센서 데이터에 대한 신뢰할 수 있는 QoS 설정

        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/agent2/points", qos, std::bind(&VoxelGridAndOutlierRemovalNode::lidar_callback, this, std::placeholders::_1));

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 100);
        pub_downsampled_ = create_publisher<sensor_msgs::msg::PointCloud2>("/voxelized_points", 100);
    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cutted(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::fromROSMsg(*input, *cloud);

        // PassThrough filter
        pcl::PassThrough<pcl::PointXYZI> cut_points;
        cut_points.setInputCloud (cloud);                //입력 
        cut_points.setFilterFieldName ("z");             //적용할 좌표 축 (eg. Z축)
        cut_points.setFilterLimits (-1.0, 1.5);          //적용할 값 (최소, 최대 값)
        cut_points.setFilterFieldName ("x");             //적용할 좌표 축 (eg. X축)
        cut_points.setFilterLimits (-10.0, 20.0);          //적용할 값 (최소, 최대 값)
        cut_points.setFilterFieldName ("y");             //적용할 좌표 축 (eg. Y축)
        cut_points.setFilterLimits (-10.0, 10.0);          //적용할 값 (최소, 최대 값)
        //cut_points.setFilterLimitsNegative (true);     //적용할 값 외 
        cut_points.filter (*cloud_cutted);             //필터 적용 

        // VoxelGrid downsampling
        pcl::VoxelGrid<pcl::PointXYZI> sor_downsample;
        sor_downsample.setInputCloud(cloud_cutted);
        sor_downsample.setLeafSize(0.5, 0.5, 0.5); // Set the voxel size
        sor_downsample.filter(*cloud_downsampled);
        // Publish the Data
        sensor_msgs::msg::PointCloud2 downsampled;
        pcl::toROSMsg(*cloud_downsampled, downsampled);
        pub_downsampled_->publish(downsampled);

        RCLCPP_INFO(this->get_logger(), "Cloud before filtering: %lu points", cloud_downsampled->size());

        // StatisticalOutlierRemoval filtering
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_filter;
        sor_filter.setInputCloud(cloud_downsampled);
        sor_filter.setMeanK(10);
        sor_filter.setStddevMulThresh(0.1);
        sor_filter.filter(*cloud_filtered);

        // Publish the Data
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        pub_->publish(output);

        RCLCPP_INFO(this->get_logger(), "Cloud after filtering: %lu points", cloud_filtered->size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_downsampled_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelGridAndOutlierRemovalNode>());
    rclcpp::shutdown();
    return 0;
}
