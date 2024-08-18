#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>

namespace simple_livo
{
    class feature_tracker
    {
    public:
        feature_tracker();

        void ReadNewFrame(const cv::Mat &frame);

        cv::Mat GetVisualizationFrame();

    private:
        void FilterVector(std::vector<cv::Point2f> &v, std::vector<uchar> &status);
        bool InBorder(const cv::Point2f &pt);

        // 关键点检测相关
        std::vector<cv::Point2f> tracker_point; //跟踪点

        cv::Mat color_frame;

        cv::Mat frame, frame_last;
        std::vector<cv::Point2f> critical_point, critical_point_last,predict_point;
        std::vector<uchar> status;
        std::vector<float> err;

        // 边缘关键点筛除参数
        int border_size; // 边缘像素大小
        int frame_col;   // 图像列数
        int frame_row;   // 图像行数

        //光流可视化相关
        bool use_visualization = true;
        cv::Mat visualization_img;
    };
}