#include <simple_livo/voxel_map/vexel_map.hpp>

namespace simple_livo
{
    vexel_map::vexel_map()
    {
        cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    }

    void vexel_map::Update()
    {
        *vexel_cloud = *cloud;
    }

    void vexel_map::SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud)
    {
        cloud = _cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vexel_map::GetInputCloud()
    {
        return vexel_cloud;
    }
}
