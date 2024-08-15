#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

class ORBDetectorNode : public rclcpp::Node
{
public:
    ORBDetectorNode() : Node("orb_detector_node")
    {
        // 创建一个订阅者，订阅图像话题
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, std::bind(&ORBDetectorNode::imageCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/orb_image", 10);

        // 初始化 ORB 检测器
        orb_ = cv::ORB::create();
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

        //更新图列表

        
        //灰度图
        cv::Mat img_gray;
        cv::cvtColor(cv_ptr->image, img_gray, cv::COLOR_BGR2GRAY);

        // 检测 ORB 关键点
        
        cv::Mat descriptors;
        orb_->detectAndCompute(img_gray, cv::noArray(), keypoints_list[0], descriptors);

        // 在图像上绘制关键点
        cv::Mat outputImage;
        cv::drawKeypoints(cv_ptr->image, keypoints_list[0], outputImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

        // 将 OpenCV 图像转换为 ROS2 图像消息
        std_msgs::msg::Header header;            // 创建一个消息头
        header.stamp = this->get_clock()->now(); // 设置时间戳

        // 使用 cv_brdge 将 OpenCV 图像转换为 ROS 图像消息
        sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(header, "bgr8", outputImage).toImageMsg();

        // 发布图像消息
        publisher_->publish(*output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    std::vector<std::vector<cv::KeyPoint>> keypoints_list = std::vector<std::vector<cv::KeyPoint>>(2);

    cv::Ptr<cv::ORB> orb_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ORBDetectorNode>());
    rclcpp::shutdown();
    return 0;
}