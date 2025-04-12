//
// Created by igs on 2023/4/11.
//

#include "poly_line_fitting.h"

namespace Fusion
{
    namespace PSE
    {

        /// find segmented point
        void DouglasPeucker::GenerateSegPoints(const int &begin,
                                               const int &end,
                                               const double &threshold)
        {
            int midPoint;
            if (end - begin > 1)
            {
                //        std::cout << "dist: " << GetMaxDist(dataIn_, begin, end) << std::endl;
                if (GetMaxDist(dataIn_, begin, end) > threshold)
                {
                    midPoint = GetMaxDistId(dataIn_, begin, end);
                    GenerateSegPoints(begin, midPoint, threshold);
                    GenerateSegPoints(midPoint, end, threshold);
                }
                else
                {
                    for (int i = begin + 1; i < end; ++i)
                    {
                        dataIn_[i]->isRemoved = true;
                    }
                }
            }
            else
            {
                return;
            }
        }

        void DouglasPeucker::Polyfit(const std::vector<Eigen::Vector3d> &points,
                                     bool &isInverse,
                                     LineParam &param)
        {
            Eigen::MatrixXd A(points.size(), 2U);
            Eigen::VectorXd yVals(points.size());
            for (int i = 0; i < points.size(); ++i)
            {
                A(i, 0) = 1.0;
                A(i, 1) = isInverse ? points[i].y() : points[i].x();
                yVals(i) = isInverse ? points[i].x() : points[i].y();
            }

            auto Q = A.householderQr();
            auto result = Q.solve(yVals);

            param.isInverse = isInverse;
            param.b = result(0);
            param.k = result(1);
            /*std::cout << "Generate new line, New line isinverse: " << isInverse << std::endl;
            if (param.isInverse) {
                std::cout << "Line: x = " << param.k << " y + " << param.b << std::endl;
            } else {
                std::cout << "Line: y = " << param.k << " x + " << param.b << std::endl;
            }*/
        }

        double DouglasPeucker::CalcPtLineDis(const Point::Ptr &p1,
                                             const Point::Ptr &p2,
                                             const Point::Ptr &p3)
        {
            double dist;
            double A, B, C;
            A = -(p2->point.y() - p1->point.y()) / (p2->point.x() - p1->point.x());
            B = 1.0;
            C = -A * p1->point.x() - p1->point.y();
            dist = abs(A * p3->point.x() + B * p3->point.y() + C) / sqrt(A * A + B * B);
            return dist;
        }

        double DouglasPeucker::GetMaxDist(std::vector<Point::Ptr> &points,
                                          const int &beginId,
                                          const int &endId)
        {
            std::vector<double> dists;
            for (int i = beginId; i < endId; ++i)
            {
                dists.emplace_back(CalcPtLineDis(points[beginId], points[endId], points[i]));
            }
            auto maxDist = std::max_element(dists.begin(), dists.end());
            return *maxDist;
        }

        int DouglasPeucker::GetMaxDistId(std::vector<Point::Ptr> &points,
                                         const int &beginId,
                                         const int &endId)
        {
            std::vector<double> dists;
            for (int i = beginId; i < endId; ++i)
            {
                dists.emplace_back(CalcPtLineDis(points[beginId], points[endId], points[i]));
            }
            auto maxDist = std::max_element(dists.begin(), dists.end());
            auto iD = points[beginId]->id + std::distance(dists.begin(), maxDist);
            return int(iD);
        }

        bool DouglasPeucker::IsInverse(const Eigen::Vector3d &startIn,
                                       const Eigen::Vector3d &endIn)
        {
            const double diffX = endIn.x() - startIn.x();
            const double diffY = endIn.y() - startIn.y();
            const double k = diffY / diffX;
            //    std::cout << "k: " << k << std::endl;

            return (std::abs(k) > 1.0);
        }

        void DouglasPeucker::Output()
        {
            for (const auto &pathPoint : dataIn_)
            {
                if (!pathPoint->isRemoved)
                {
                    /*  std::cout << "Find segmented point id: " << pathPoint->id
                                << " | " << pathPoint->point.x()
                                << " " << pathPoint->point.y()
                                << " " << pathPoint->point.z() << std::endl;*/
                    allSegPoints_.emplace_back(pathPoint);
                }
            }

            for (int i = 0; i < int(allSegPoints_.size() - 1); ++i)
            {
                std::vector<Eigen::Vector3d> oneLineSegPoints(2);
                oneLineSegPoints[0] = allSegPoints_[i]->point;
                oneLineSegPoints[1] = allSegPoints_[i + 1]->point;
                bool isInverse = IsInverse(oneLineSegPoints[0], oneLineSegPoints[1]);
                LineParam lineParam;
                Polyfit(oneLineSegPoints, isInverse, lineParam);
                if (!isInverse)
                {
                    lineParam.start = oneLineSegPoints[0].x();
                    lineParam.end = oneLineSegPoints[1].x();
                    ///@debug
                    //            std::cout << "Line: y= " << lineParam.k << " x+ " << lineParam.b << std::endl;
                    ///@debug end
                }
                else
                {
                    lineParam.start = oneLineSegPoints[0].y();
                    lineParam.end = oneLineSegPoints[1].y();
                    ///@debug
                    //            std::cout << "Line: x= " << lineParam.k << " y+ " << lineParam.b << std::endl;
                    ///@debug end
                }
                lineParams_.emplace_back(lineParam);
            }
        }

        std::vector<LineParam> DouglasPeucker::GetParams()
        {
            return lineParams_;
        }

        void DouglasPeucker::Init(const std::vector<Point::Ptr> &dataIn)
        {
            dataIn_ = dataIn;
            /// reset point id and point isRemoved
            for (int i = 0; i < int(dataIn_.size()); ++i)
            {
                dataIn_[i]->id = i;
                dataIn_[i]->isRemoved = false;
            }
        }
    }
}