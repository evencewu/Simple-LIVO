#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>



#define FRAME_NOW 0
#define FRAME_LAST 1

namespace simple_livo
{
    class VoPredict
    {
    public:

        void Init();
        void Update(const cv::Mat &frame);

        void DetectFeatures();
        void CalcOpticalFlow();

        std::vector<cv::KeyPoint> DetectOrb();

        cv::Mat GetVisualizationFrame();
        Eigen::Affine3d GetTransformationMat();

    private:
        std::vector<cv::Point2f> critical_points[2];
        std::vector<cv::Point2f> useful_points[2];

        cv::Mat frame[2];
        cv::Mat gray_frame[2];

        std::vector<uchar> status;
        std::vector<float> err;

        Eigen::Affine3d pose = Eigen::Affine3d::Identity();

        //output_frame
    };
}