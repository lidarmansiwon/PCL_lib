#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>

class VoxelGridAndOutlierRemovalNode : public rclcpp::Node
{
public:
    VoxelGridAndOutlierRemovalNode()
        : Node("VoxelGridAndOutlierRemoval")
    {
        // Declare parameters with default values
        declare_parameters();

        // Get parameter values
        get_parameters();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); // 센서 데이터에 대한 신뢰할 수 있는 QoS 설정

        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", rclcpp::SensorDataQoS(), std::bind(&VoxelGridAndOutlierRemovalNode::lidar_callback, this, std::placeholders::_1));

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 100);
        pub_downsampled_ = create_publisher<sensor_msgs::msg::PointCloud2>("/voxelized_points", 100);
    }

private:

    double crop_box_z_min_;
    double crop_box_z_max_;
    double crop_box_x_min_;
    double crop_box_x_max_;
    double crop_box_y_min_;
    double crop_box_y_max_;
    double voxel_resolution_;
    double setMean_;
    double setStddevMulThresh_;
    

    void declare_parameters()
    {   
        declare_parameter<double>("crop_box_y_min", -10.0);
        declare_parameter<double>("crop_box_y_max", 10.0);
        declare_parameter<double>("crop_box_z_min", -1.0);
        declare_parameter<double>("crop_box_z_max", 1.5);
        declare_parameter<double>("crop_box_x_min", -10.0);
        declare_parameter<double>("crop_box_x_max", 20.0);

        declare_parameter<double>("voxel_resolution", 0.5);
        declare_parameter<double>("setMean", 10.0);
        declare_parameter<double>("setStddevMulThresh", 1.0);
    }

    void get_parameters()
    {   
        get_parameter("crop_box_y_min", crop_box_y_min_);
        get_parameter("crop_box_y_max", crop_box_y_max_);
        get_parameter("crop_box_z_min", crop_box_z_min_);
        get_parameter("crop_box_z_max", crop_box_z_max_);
        get_parameter("crop_box_x_min", crop_box_x_min_);
        get_parameter("crop_box_x_max", crop_box_x_max_);

        get_parameter("voxel_resolution", voxel_resolution_);
        get_parameter("setMean", setMean_);
        get_parameter("setStddevMulThresh", setStddevMulThresh_);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr applyCropBoxFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
    {
        pcl::CropBox<pcl::PointXYZI> crop_box;
        crop_box.setInputCloud(input_cloud);
        crop_box.setMin(Eigen::Vector4f(crop_box_x_min_, crop_box_y_min_, crop_box_z_min_, 1.0));
        crop_box.setMax(Eigen::Vector4f(crop_box_x_max_, crop_box_y_max_, crop_box_z_max_, 1.0));

        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        crop_box.filter(*output_cloud);

        return output_cloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr convertPointCloud2ToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);
    return cloud;
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

        // pcl::fromROSMsg(*input, *cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = convertPointCloud2ToPCL(input);

        // CropBox filter
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cropped = applyCropBoxFilter(cloud);


        // VoxelGrid downsampling
        pcl::VoxelGrid<pcl::PointXYZI> sor_downsample;
        sor_downsample.setInputCloud(cloud_cropped);
        sor_downsample.setLeafSize(voxel_resolution_, voxel_resolution_, voxel_resolution_); // Set the voxel size
        sor_downsample.filter(*cloud_downsampled);
        // Publish the Data
        sensor_msgs::msg::PointCloud2 downsampled;
        pcl::toROSMsg(*cloud_downsampled, downsampled);
        pub_downsampled_->publish(downsampled);

        // RCLCPP_INFO(this->get_logger(), "Cloud before filtering: %lu points", cloud_downsampled->size());

        // StatisticalOutlierRemoval filtering
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_filter;
        sor_filter.setInputCloud(cloud_downsampled);
        sor_filter.setMeanK(setMean_);
        sor_filter.setStddevMulThresh(setStddevMulThresh_);
        sor_filter.filter(*cloud_filtered);

        // Publish the Data
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        pub_->publish(output);

        // RCLCPP_INFO(this->get_logger(), "Cloud after filtering: %lu points", cloud_filtered->size());
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
