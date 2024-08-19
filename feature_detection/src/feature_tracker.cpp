#include "feature_detection/feature_tracker.hpp"

namespace simple_livo
{
    feature_tracker::feature_tracker()
    {
    }

    bool feature_tracker::InBorder(const cv::Point2f &pt)
    {
        const int border_size = 1;
        int img_x = cvRound(pt.x);
        int img_y = cvRound(pt.y);
        return border_size <= img_x && img_x < frame_col - border_size && border_size <= img_y && img_y < frame_row - border_size;
    }

    void feature_tracker::FilterVector(std::vector<cv::Point2f> &v, std::vector<uchar> &status)
    {
        int j = 0;
        for (int i = 0; i < int(v.size()); i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);
    }

    void feature_tracker::ReadNewFrame(const cv::Mat &_frame)
    {
        color_frame = _frame;
        cv::cvtColor(_frame, frame, cv::COLOR_BGR2GRAY);

        frame_col = _frame.cols;
        frame_row = _frame.rows;

        if (frame_last.empty())
        {
            RCLCPP_INFO(rclcpp::get_logger("TestNode"), "检测角点");
            cv::goodFeaturesToTrack(frame, critical_point, 100, 0.01, 10); // 检测角点
        }
        else
        {

            if (critical_point_last.size() < 6)
            {
                cv::goodFeaturesToTrack(frame_last, critical_point_last, 100, 0.01, 10); // 检测角点
            }

            cv::calcOpticalFlowPyrLK(frame_last, frame, critical_point_last, critical_point, status, err, cv::Size(21, 21), 3);

            for (int i = 0; i < int(critical_point.size()); i++)
                if (status[i] && !InBorder(critical_point[i]))
                    status[i] = 0;

            FilterVector(critical_point_last, status);
            FilterVector(critical_point, status);

            if (use_visualization)
            {
                _frame.copyTo(visualization_img);
                SetVisualizationFrame(visualization_img);

                // 在图像上绘制圆点

                /*
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
                */

                // std::stringstream ss;

                if (!critical_point.empty())
                {
                    RCLCPP_INFO(rclcpp::get_logger("TestNode"), "%f %f", critical_point[0].x, critical_point[0].y);
                    RCLCPP_INFO(rclcpp::get_logger("TestNode"), "%ld %ld", size(critical_point), size(critical_point_last));
                }
            }
        }

        critical_point_last = critical_point;
        frame.copyTo(frame_last);
    }

    void feature_tracker::SetVisualizationFrame(const cv::Mat &frame)
    {
        for (const auto &point : critical_point_last)
        {
            cv::circle(frame, point, 5, cv::Scalar(0, 255, 0), -1);
        }

        for (const auto &point : critical_point)
        {
            cv::circle(frame, point, 3, cv::Scalar(0, 0, 255), -1);
        }
    }

    cv::Mat feature_tracker::GetVisualizationFrame()
    {
        return visualization_img;
    }
}