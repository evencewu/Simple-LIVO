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

#include <simple_livo/voxel_map/vexel_map.hpp>

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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

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

        /*
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // Fill in the cloud data
        pcl::PCDReader reader;
        // Replace the path below with the path where you saved your file
        reader.read<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud);

        std::cerr << "Cloud before filtering: " << std::endl;
        std::cerr << *cloud << std::endl;

        // Create the filtering object
        // 创建滤波器对象，及相关参数设置
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor; // 创建对象
        sor.setInputCloud(cloud);                          // 设置输入点云
        sor.setMeanK(50);                                  // 设置统计时考虑查询点邻近点数
        
        //设置判断是否为离群点的阈值
        //更具体为设置标准差倍数阈值 std_mul ，点云中所有点与其邻域的距离大于 μ ±σ• std_mul
        //则被认为是离群点，其中 μ 代表估计的平均距离， σ 代表标准差 。
         
        sor.setStddevMulThresh(1.0); // 设置为1代表：如果一个点的距离超过平均距离一个标准差以上，则会被当做离群点去除
        sor.filter(*cloud_filtered); // 执行滤波，并将结果保存在cloud_filter中

        std::cerr << "Cloud after filtering: " << std::endl;
        std::cerr << *cloud_filtered << std::endl;

        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

        sor.setNegative(true); // 选择被过滤以外的点，即离群点
        sor.filter(*cloud_filtered);
        writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

        */

        // 应用体素栅格滤波
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(pcl_cloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        vg.filter(*cloud_voxel_filtered);

        // 转换回 ROS 2 消息并发布
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_voxel_filtered, output_msg);
        output_msg.header.frame_id = "/camera_link";

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