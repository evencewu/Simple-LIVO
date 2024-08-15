#include "attitude_prediction/VoPredict.hpp"

namespace simple_livo
{
    void VoPredict::Update(const cv::Mat &frame)
    {
        //滚动数据
        this->frame[FRAME_LAST] = this->frame[FRAME_NOW];
        gray_frame[FRAME_LAST] = gray_frame[FRAME_NOW];
        critical_points[FRAME_LAST] = critical_points[FRAME_NOW];

        this->frame[FRAME_NOW] = frame;
        cv::cvtColor(this->frame[FRAME_NOW], gray_frame[FRAME_NOW], cv::COLOR_BGR2GRAY);

        //角点提取
        DetectFeatures();
        //光流筛选
        CalcOpticalFlow();
    }

    void VoPredict::DetectFeatures()
    {
        cv::goodFeaturesToTrack(gray_frame[FRAME_NOW], critical_points[FRAME_NOW], 100, 0.01, 10); // 检测角点
    }

    void VoPredict::CalcOpticalFlow()
    {
        if (!critical_points[FRAME_LAST].empty())
        {
            // 计算光流
            std::vector<cv::Point2f> points_predict;

            cv::calcOpticalFlowPyrLK(gray_frame[FRAME_LAST], gray_frame[FRAME_NOW],
                                     critical_points[FRAME_LAST], points_predict, status, err);

            // 筛选成功跟踪的点

            std::vector<cv::Point2f> trackedPoints1, trackedPoints2;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                {
                    trackedPoints1.push_back(critical_points[FRAME_LAST][i]);
                    trackedPoints2.push_back(points_predict[i]);
                }
            }

            useful_points[FRAME_LAST] = trackedPoints1;
            useful_points[FRAME_NOW] = trackedPoints2;

            if (trackedPoints1.size() >= 5)
            {
                // 假设相机的内参矩阵 (这里使用一个简单的假设内参)
                cv::Mat K = (cv::Mat_<double>(3, 3) << 718.8560, 0, 607.1928, 0, 718.8560, 185.2157, 0, 0, 1);

                // 计算本质矩阵
                cv::Mat essential_matrix = cv::findEssentialMat(trackedPoints1, trackedPoints2, K, cv::RANSAC);

                // 从本质矩阵恢复相机姿态 (旋转和平移)
                cv::Mat R, t;
                cv::recoverPose(essential_matrix, trackedPoints1, trackedPoints2, K, R, t);

                // 将 OpenCV 的旋转矩阵 R 转换为 Eigen 的矩阵
                Eigen::Matrix3d rotation_matrix;
                cv::cv2eigen(R, rotation_matrix);

                // 将 OpenCV 的平移向量 t 转换为 Eigen 的向量
                Eigen::Vector3d translation_vector;
                cv::cv2eigen(t, translation_vector);

                // 设置旋转和平移
                pose.linear() = rotation_matrix;         // 设置旋转部分 (3x3 矩阵)
                pose.translation() = translation_vector; // 设置平移部分 (3x1 向量)
            }
        }
    }

    cv::Mat VoPredict::GetVisualizationFrame()
    {
        // 创建图像的副本，以便不修改原始图像
        cv::Mat output_img;
        frame[0].copyTo(output_img);

        // 在图像上绘制圆点
        for (const auto &point : useful_points[FRAME_LAST])
        {
            cv::circle(output_img, point, 7, cv::Scalar(0, 255, 0), -1);
        }

        for (const auto &point : useful_points[FRAME_NOW])
        {
            cv::circle(output_img, point, 5, cv::Scalar(255, 0, 0), -1);
        }

        for (size_t i = 0; i < status.size(); i++)
        {
            cv::line(output_img, useful_points[FRAME_LAST][i], useful_points[FRAME_NOW][i], cv::Scalar(255, 0, 0), 2);
        }

        return output_img;
    }

    Eigen::Affine3d VoPredict::GetTransformationMat()
    {
        return pose;
    }
}