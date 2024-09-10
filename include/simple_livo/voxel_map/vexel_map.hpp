#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/bilateral.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

namespace simple_livo
{
    class vexel_map
    {
    public:
        vexel_map();

        void Update();
        void NoiseFiltering();

        void SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetOutputCloud();

    protected:
        

    private:
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filted_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr vexel_cloud;

    };
}