#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/bilateral.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

class PointCloudSubscriber : public rclcpp::Node
{
public:
    PointCloudSubscriber() : Node("pointcloud_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10, std::bind(&PointCloudSubscriber::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud", 10);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message");

        // 将PointCloud2消息转换为PCL格式
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // 转换 ROS 2 的 PointCloud2 消息为 PCL 点云
        // pcl::PCLPointCloud2 pcl_pc2;
        // pcl_conversions::toPCL(*msg, pcl_pc2);
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>());

        // pcl::copyPointCloud(*cloud, *cloud_xyzi);

        // 双边滤波

        // Set up KDTree
        // pcl::PointCloud<pcl::PointXYZI> cloud_filtered;

        // pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
        // pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);

        // pcl::PointCloud<pcl::PointXYZI> outcloud;

        // pcl::BilateralFilter<pcl::PointXYZI> bf;
        // bf.setInputCloud(cloud_xyzi);
        // bf.setSearchMethod(tree);
        // bf.setHalfSize(0.3);
        // bf.setStdDev(0.1);
        // bf.filter(outcloud);

        // 应用体素栅格滤波
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::VoxelGrid<pcl::PointXYZ> vg;
        // vg.setInputCloud(cloud);
        // vg.setLeafSize(0.01f, 0.01f, 0.01f);
        // vg.filter(*cloud_voxel_filtered);

        // 转换回 ROS 2 消息并发布
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(pcl_cloud, output_msg);
        publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSubscriber>());
    rclcpp::shutdown();
    return 0;
}