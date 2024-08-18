#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>

class OneTimeSubscriber : public rclcpp::Node
{
public:
    OneTimeSubscriber(const std::string img_name) : Node("one_time_subscriber"),img_name(img_name)
    {
        
        RCLCPP_INFO(rclcpp::get_logger("TestNode"), "Init");
        // 创建一个订阅者，订阅图像话题
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, std::bind(&OneTimeSubscriber::imageCallback, this, std::placeholders::_1));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("TestNode"), "Callback");
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

        // 保存图像为 PNG 格式
        cv::imwrite(img_name + ".png", cv_ptr->image);

        RCLCPP_INFO(rclcpp::get_logger("TestNode"), "Image saved successfully!");

        rclcpp::shutdown();
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    std::string img_name = "base";
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 创建一个OneTimeSubscriber节点
    RCLCPP_INFO(rclcpp::get_logger("TestNode"), "Name is %s",argv[1]);
    auto node = std::make_shared<OneTimeSubscriber>(argv[1]);

    // 节点运行直到接收到一次消息后自动关闭
    rclcpp::spin(node);

    // 清理并关闭ROS 2节点
    rclcpp::shutdown();

    return 0;
}

/*
cv::Mat loaded_image = cv::imread("saved_image.png", cv::IMREAD_COLOR);

    if(loaded_image.empty()) {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    std::cout << "Image loaded successfully!" << std::endl;

    // 显示图像
    cv::imshow("Loaded Image", loaded_image);
    cv::waitKey(0);
    */