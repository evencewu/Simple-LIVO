#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "feature_detection/feature_tracker.hpp"

class TestDetectorNode : public rclcpp::Node
{
public:
    TestDetectorNode() : Node("test_detector_node")
    {
        // 创建一个订阅者，订阅图像话题
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, std::bind(&TestDetectorNode::imageCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/test_image", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {

        // 将 ROS 图像消息转换为 OpenCV 图像
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        vo.ReadNewFrame(cv_ptr->image);
        cv::Mat outputImage = vo.GetVisualizationFrame();

        /*
        std::stringstream ss;
        ss << "Transformation matrix: " << std::endl
           << vo.GetTransformationMat().matrix() << std::endl;

        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        */
        
        // 将 OpenCV 图像转换为 ROS2 图像消息
        std_msgs::msg::Header header;            // 创建一个消息头
        header.stamp = this->get_clock()->now(); // 设置时间戳

        // 使用 cv_brdge 将 OpenCV 图像转换为 ROS 图像消息
        sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(header, "bgr8", outputImage).toImageMsg();

        // 发布图像消息
        publisher_->publish(*output_msg);
    }

    simple_livo::feature_tracker vo;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    std::vector<std::vector<cv::Point2f>> keypoints_list = std::vector<std::vector<cv::Point2f>>(2);

    cv::Ptr<cv::ORB> orb_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestDetectorNode>());
    rclcpp::shutdown();
    return 0;
}