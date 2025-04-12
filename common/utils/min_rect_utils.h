#ifndef MIN_RECT_UTILS
#define MIN_RECT_UTILS

#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <array>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/opencv.hpp>

namespace Fusion
{

    class MinAreaRect
    {

    public:
        MinAreaRect(std::vector<Eigen::Vector3d> edgePoints)
        {
            this->edgePoints_ = edgePoints;
        }

        bool Solve(std::array<Eigen::Vector3d, 4U> &minAreaRect)
        {
            if (this->edgePoints_.empty())
            {
                return false;
            }
            Eigen::Vector3d center = this->GetCenter(edgePoints_);
            double minArea = 1e20;
            double minTheta = 0.;
            std::array<Eigen::Vector3d, 4U> simpleRect;
            this->GetSimpleRect(this->edgePoints_, simpleRect);
            std::array<Eigen::Vector3d, 4U> minSimpleRect(simpleRect);
            for (uint32_t angle = 0U; angle < 180U; ++angle)
            {
                std::vector<Eigen::Vector3d> rEdgePoints{};
                double theta = (static_cast<double>(angle) * 1.0) / (180. * M_PI);
                this->Rotate(edgePoints_, rEdgePoints, theta, center);
                std::array<Eigen::Vector3d, 4U> rSimpleRect;
                this->GetSimpleRect(rEdgePoints, rSimpleRect);
                double area = this->GerRectArea(rSimpleRect);
                if (area < minArea)
                {
                    minArea = area;
                    minTheta = theta;
                    minSimpleRect = rSimpleRect;
                }
            }

            std::vector<Eigen::Vector3d> slovedRect(minSimpleRect.begin(), minSimpleRect.end());
            std::vector<Eigen::Vector3d> roatedRect{};
            this->Rotate(slovedRect, roatedRect, -minTheta, center);
            for (uint32_t i = 0U; i < 4U; i++)
            {
                minAreaRect[i] = roatedRect[i];
            }
            return true;
        }

    private:
        Eigen::Vector3d GetCenter(const std::vector<Eigen::Vector3d> &points)
        {
            Eigen::Vector3d center{0., 0., 0.};
            for (const auto &point : points)
            {
                center += point;
            }
            center /= points.size();
            return center;
        }

        bool GetSimpleRect(const std::vector<Eigen::Vector3d> &edgePoints, std::array<Eigen::Vector3d, 4U> &rect)
        {
            if (edgePoints.empty())
            {
                return false;
            }
            double xMax = -1e10;
            double yMax = -1e10;
            double xMin = +1e10;
            double yMin = +1e10;
            for (const auto &point : edgePoints)
            {
                xMax = std::max(xMax, point.x());
                yMax = std::max(yMax, point.y());
                xMin = std::min(xMin, point.x());
                yMin = std::min(yMin, point.y());
            }
            rect[0] = {xMin, yMin, 0.};
            rect[1] = {xMin, yMax, 0.};
            rect[2] = {xMax, yMax, 0.};
            rect[3] = {xMax, yMin, 0.};
            return true;
        }

        double GerRectArea(std::array<Eigen::Vector3d, 4U> &rect)
        {
            double deltaY = (rect[0U] - rect[1U]).norm();
            double deltaX = (rect[1U] - rect[2U]).norm();
            return deltaY * deltaX;
        }

        void Rotate(const std::vector<Eigen::Vector3d> &edgePoints,
                    std::vector<Eigen::Vector3d> &rEdgePoints, double theta, const Eigen::Vector3d &rCenter)
        {
            double xCenter = rCenter.x();
            double yCenter = rCenter.y();

            for (const auto &point : edgePoints)
            {
                double x = point.x();
                double y = point.y();
                double rX = (x - xCenter) * cos(theta) - (y - yCenter) * sin(theta) + xCenter;
                double rY = (x - xCenter) * sin(theta) + (y - yCenter) * cos(theta) + yCenter;
                Eigen::Vector3d rPoint = {rX, rY, 0.};
                rEdgePoints.push_back(rPoint);
            }
        }

    private:
        std::vector<Eigen::Vector3d> edgePoints_;
    };

    class MinAreaRectCV
    {

    public:
        MinAreaRectCV(std::vector<Eigen::Vector3d> edgePoints)
        {
            this->edgePoints_ = edgePoints;
        }

        bool Solve(std::array<Eigen::Vector3d, 4U> &minAreaRect)
        {
            std::vector<cv::Point2f> cvEdgePoints;
            for (const auto &edgePoint : edgePoints_)
            {
                cvEdgePoints.push_back(cv::Point2f(static_cast<float>(edgePoint.x()), static_cast<float>(edgePoint.y())));
            }
            cv::RotatedRect mar = cv::minAreaRect(cvEdgePoints);
            cv::Point2f rectPoints[4];
            // must be recieve float type.
            mar.points(rectPoints);
            for (size_t i = 0; i < 4U; ++i)
            {
                float x = rectPoints[i].x;
                float y = rectPoints[i].y;
                minAreaRect[i] = {static_cast<double>(x), static_cast<double>(y), 0.0};
            }
            return true;
        }

    private:
        std::vector<Eigen::Vector3d> edgePoints_;
    };

}
#endif // MIN_RECT_UTILS
