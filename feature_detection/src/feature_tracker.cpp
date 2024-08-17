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

    void feature_tracker::FilterVector(std::vector<cv::Point2f> &v, std::vector<uchar> status)
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

        if (!frame_last.empty())
        {
            cv::calcOpticalFlowPyrLK(frame_last, frame, critical_point_last, predict_point, status, err, cv::Size(21, 21), 3);
        }

        cv::goodFeaturesToTrack(frame, critical_point, 100, 0.01, 10); // 检测角点

        if (use_visualization)
        {
            visualization_img = _frame;

            // 在图像上绘制圆点
            for (const auto &point : critical_point_last)
            {
                cv::circle(visualization_img, point, 7, cv::Scalar(0, 255, 0), -1);
            }

            for (const auto &point : predict_point)
            {
                cv::circle(visualization_img, point, 3, cv::Scalar(0, 0, 255), -1);
            }

            if (!predict_point.empty())
            {
                RCLCPP_INFO(rclcpp::get_logger("TestNode"), "%f %f %f %f", predict_point[0].x, predict_point[0].y, critical_point_last[0].x, critical_point_last[0].y);
                RCLCPP_INFO(rclcpp::get_logger("TestNode"), "%ld %ld ", size(predict_point), size(critical_point_last));
            }
        }

        critical_point_last = critical_point;
        frame_last = frame;
    }

    cv::Mat feature_tracker::GetVisualizationFrame()
    {
        /*
        // 创建图像的副本，以便不修改原始图像
        cv::Mat output_img;
        color_frame.copyTo(output_img);

        // 在图像上绘制圆点
        for (const auto &point : critical_point_last)
        {
            cv::circle(output_img, point, 7, cv::Scalar(0, 255, 0), -1);
        }

        for (const auto &point : predict_point)
        {
            cv::circle(output_img, point, 3, cv::Scalar(0, 0, 255), -1);
        }

        if (!predict_point.empty())
        {
            RCLCPP_INFO(rclcpp::get_logger("TestNode"), "%f %f %f %f", predict_point[0].x, predict_point[0].y, critical_point_last[0].x, critical_point_last[0].y);
            RCLCPP_INFO(rclcpp::get_logger("TestNode"), "%ld %ld ", size(predict_point), size(critical_point_last));
        }

        // if (!critical_point_last.empty() && !predict_point.empty())
        //{
        //     cv::line(output_img, critical_point_last[0], predict_point[0], cv::Scalar(255, 0, 0), 2);
        // }

        // for (size_t i = 0; i < status.size(); i++)
        //{
        //     cv::line(output_img, critical_point_last[i], predict_point[i], cv::Scalar(255, 0, 0), 2);
        // }
        */

        return visualization_img;
    }

}