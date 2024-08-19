#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

class TestDetectorNode : public rclcpp::Node
{
public:
    bool InBorder(const cv::Point2f &pt)
    {
        const int border_size = 1;
        int img_x = cvRound(pt.x);
        int img_y = cvRound(pt.y);
        return border_size <= img_x && img_x < frame_col - border_size && border_size <= img_y && img_y < frame_row - border_size;
    }

    void FilterVector(std::vector<cv::Point2f> &v, std::vector<uchar> &status)
    {
        int j = 0;
        for (int i = 0; i < int(v.size()); i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);
    }

    TestDetectorNode() : Node("test_detector_node")
    {
        // ros
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/test_image", 10);

        // img

        // 从文件中读取图像
        cv::Mat image_1 = cv::imread("/home/orthrus/orthrus/base1.png", cv::IMREAD_COLOR);
        cv::Mat image_2 = cv::imread("/home/orthrus/orthrus/base2.png", cv::IMREAD_COLOR);

        frame_col = image_1.cols;
        frame_row = image_1.rows;

        if (!image_1.empty() && !image_2.empty())
        {
            cv::cvtColor(image_1, frame_1, cv::COLOR_BGR2GRAY);
            cv::cvtColor(image_2, frame_2, cv::COLOR_BGR2GRAY);

            cv::goodFeaturesToTrack(frame_1, point1, 100, 0.01, 10); // 检测角点

            cv::calcOpticalFlowPyrLK(frame_1, frame_2, point1, point2, status, err, cv::Size(21, 21), 3);

            for (int i = 0; i < int(point2.size()); i++)
                if (status[i] && !InBorder(point2[i]))
                    status[i] = 0;

            FilterVector(point1, status);
            FilterVector(point2, status);

            RCLCPP_INFO(rclcpp::get_logger("TestNode"), "sizeof point1: %ld", point1.size());
            RCLCPP_INFO(rclcpp::get_logger("TestNode"), "sizeof point2: %ld", point2.size());

            for (const auto &point : point1)
            {
                cv::circle(image_1, point, 7, cv::Scalar(0, 255, 0), -1);
            }

            for (const auto &point : point2)
            {
                cv::circle(image_2, point, 3, cv::Scalar(0, 0, 255), -1);
            }

            for (size_t i = 0; i < status.size(); i++)
            {
                cv::line(image_2, point1[i], point2[i], cv::Scalar(255, 0, 0), 2);
                cv::line(image_1, point1[i], point2[i], cv::Scalar(255, 0, 0), 2);
            }

            // 相机内参
            cv::Mat K = (cv::Mat_<double>(3, 3) << 718.856, 0, 607.1928,
                         0, 718.856, 185.2157,
                         0, 0, 1);

            // 计算本质矩阵
            cv::Mat essential_matrix = cv::findEssentialMat(point1, point2, K, cv::RANSAC);

            // 从本质矩阵中恢复相机位姿
            cv::Mat R, t;
            cv::recoverPose(essential_matrix, point1, point2, K, R, t);

            // 输出结果
            std::stringstream ss;
            ss << "Rotation Matrix: " << std::endl
               << R << std::endl;
            ss << "Translation Vector: " << std::endl
               << t << std::endl;

            RCLCPP_INFO(rclcpp::get_logger("TestNode"), "%s", ss.str().c_str());

            // 将 OpenCV 图像转换为 ROS2 图像消息
            // std_msgs::msg::Header header;            // 创建一个消息头
            // header.stamp = this->get_clock()->now(); // 设置时间戳

            // 使用 cv_brdge 将 OpenCV 图像转换为 ROS 图像消息
            // sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(header, "bgr8", outputImage).toImageMsg();

            // 保存图像为 PNG 格式
            cv::imwrite("output2.png", image_2);
            cv::imwrite("output1.png", image_1);

            RCLCPP_INFO(rclcpp::get_logger("TestNode"), "Image saved successfully!");

            // 发布图像消息
            // publisher_->publish(*output_msg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open or find the image");
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    cv::Mat frame_1, frame_2;
    std::vector<cv::Point2f> point1, point2;
    std::vector<uchar> status;
    std::vector<float> err;

    int border_size = 1;
    int frame_col, frame_row;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestDetectorNode>());
    rclcpp::shutdown();
    return 0;
}