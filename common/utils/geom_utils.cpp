// Copyright (C) 2022 SAIC PP-CEM. All Rights Reserved.
// LEGAL NOTICE: All information contained herein is, and remains the property of SAIC PP-CEM.
#include "geom_utils.h"

namespace Fusion
{
    namespace Geometry
    {

        StorageType Point2d2Vector3d(const BoostPoint point)
        {
            return StorageType(point.get<0>(), point.get<1>(), 0.0);
        }

        BoostPoint MakeBoostPoint(COORDTYPE x, COORDTYPE y)
        {
            BoostPoint point{x, y};
            // pointVector.z()
            return point;
        }

        BoostPoint MakeBoostPoint(const StorageType &pointVector)
        {
            BoostPoint point{pointVector.x(), pointVector.y()};
            // pointVector.z()
            return point;
        }

        BoostMultiPoint MakeBoostMultiPoint(const std::vector<StorageType> &vectors)
        {
            BoostMultiPoint multiPoint;
            for (const auto &pointVector : vectors)
            {
                multiPoint.push_back(BoostPoint(pointVector.x(), pointVector.y()));
            }
            // pointVector.z()
            return multiPoint;
        }

        BoostPolygon MakeBoostPolygon(const std::vector<StorageType> &vectors)
        {
            BoostPolygon poly;
            for (const auto &pointVector : vectors)
            {
                poly.outer().push_back(BoostPoint(pointVector.x(), pointVector.y()));
            }
            // pointVector.z()
            return poly;
        }

        BoostPolygon MakeBoostPolygon(const std::vector<BoostPoint> &vectors)
        {
            BoostPolygon poly;
            for (const auto &point : vectors)
            {
                poly.outer().push_back(BoostPoint(point));
            }
            // pointVector.z()
            return poly;
        }

        BoostPolygon MakeConvexHull(const BoostPolygon &poly)
        {
            BoostPolygon convexHull;
            bg::convex_hull(poly, convexHull);
            return convexHull;
        }

        BoostPolygon MakeConvexHull(const BoostMultiPoint &points)
        {
            BoostPolygon convexHull;
            bg::convex_hull(points, convexHull);
            return convexHull;
        }

        void MakeConvexHull(const std::vector<StorageType> &vectors, std::vector<StorageType> &convexHull)
        {
            BoostPolygon poly = Geometry::MakeBoostPolygon(vectors);
            BoostPolygon ConvexHullPoly = MakeConvexHull(poly);
            for (const auto &point : ConvexHullPoly.outer())
            {
                convexHull.push_back({point.get<0>(), point.get<1>(), 0.0});
            }
        }

        BoostBox MakeBoostBox(const std::pair<BoostPoint, BoostPoint> vectors)
        {
            BoostBox box{vectors.first, vectors.second};
            return box;
        }

        BoostLine MakeBoostLine(const std::vector<StorageType> &vectors)
        {
            BoostLine line;
            for (const auto &point : vectors)
            {
                line.push_back(MakeBoostPoint(point));
            }
            return line;
        }

        void MakeBuffer(const BoostPoint &point, BoostMultiPolygon &outPoly,
                        const COORDTYPE buffer_distance, const uint32_t points_per_circle)
        {
            // Declare strategies
            bg::strategy::buffer::distance_symmetric<COORDTYPE> distance_strategy(buffer_distance);
            bg::strategy::buffer::join_round join_strategy(points_per_circle);
            bg::strategy::buffer::end_round end_strategy(points_per_circle);
            bg::strategy::buffer::point_circle circle_strategy(points_per_circle);
            bg::strategy::buffer::side_straight side_strategy;
            bg::buffer(point, outPoly, distance_strategy, side_strategy,
                       join_strategy, end_strategy, circle_strategy);
        }

        void MakeBuffer(const BoostPolygon &inPoly, BoostMultiPolygon &outPoly,
                        const COORDTYPE buffer_distance, const uint32_t points_per_circle)
        {
            // Declare strategies
            bg::strategy::buffer::distance_symmetric<COORDTYPE> distance_strategy(buffer_distance);
            bg::strategy::buffer::join_round join_strategy(points_per_circle);
            bg::strategy::buffer::end_round end_strategy(points_per_circle);
            bg::strategy::buffer::point_circle circle_strategy(points_per_circle);
            bg::strategy::buffer::side_straight side_strategy;
            bg::buffer(inPoly, outPoly, distance_strategy, side_strategy,
                       join_strategy, end_strategy, circle_strategy);
        }

        void MakeBuffer(const BoostLine &line, BoostMultiPolygon &outPoly,
                        const COORDTYPE buffer_distance, const uint32_t points_per_circle)
        {
            // Declare strategies
            bg::strategy::buffer::distance_symmetric<COORDTYPE> distance_strategy(buffer_distance);
            bg::strategy::buffer::join_round join_strategy(points_per_circle);
            bg::strategy::buffer::end_round end_strategy(points_per_circle);
            bg::strategy::buffer::point_circle circle_strategy(points_per_circle);
            bg::strategy::buffer::side_straight side_strategy;
            bg::buffer(line, outPoly, distance_strategy, side_strategy,
                       join_strategy, end_strategy, circle_strategy);
        }

        void BoostPolygonDiff(const BoostPolygon &p1, const BoostPolygon &p2, BoostMultiPolygon &outPoly)
        {
            bg::sym_difference(p1, p2, outPoly);
        }

        bool IsCross(const BoostPoint &point, const BoostPolygon &poly)
        {
            return bg::crosses(point, poly);
        }

        bool IsCross(const BoostLine &line, const BoostPolygon &poly)
        {
            return bg::crosses(line, poly);
            ;
        }

        bool IsIntersects(const BoostPoint &point, const BoostPolygon &poly)
        {
            return bg::intersects(point, poly);
        }

        double IntersectArea(BoostPolygon poly1, BoostPolygon poly2)
        {
            std::vector<BoostPolygon> outPolys;
            bg::intersection(poly1, poly2, outPolys);
            double area = 0.0;
            for (const auto &poly : outPolys)
            {
                area += abs(bg::area(poly));
            }
            return area;
        }

        std::vector<StorageType> Intersectioin(const std::vector<StorageType> polyPoints1,
                                               const std::vector<StorageType> polyPoints2)
        {
            BoostPolygon poly1 = MakeBoostPolygon(polyPoints1);
            BoostPolygon poly2 = MakeBoostPolygon(polyPoints2);
            std::vector<BoostPolygon> outPolys;
            bg::intersection(poly1, poly2, outPolys);

            double maxArea = 0.0;
            BoostPolygon ipoly;
            for (const auto &poly : outPolys)
            {
                // std::cout << "*********" << std::endl;
                double area = abs(bg::area(poly));
                if (area > maxArea)
                {
                    maxArea = area;
                    ipoly = poly;
                }
                // for(const auto& point :  poly.outer()) {
                //     std::cout << bg::dsv(point) << std::endl;
                // }
            }

            std::vector<StorageType> ipolyVectors{};
            for (const auto &point : ipoly.outer())
            {
                ipolyVectors.push_back({point.get<0>(), point.get<1>(), 0.0});
            }
            return ipolyVectors;
        }

        double IntersectArea(const std::vector<StorageType> polyPoints1,
                             const std::vector<StorageType> polyPoints2)
        {
            BoostPolygon poly1 = MakeBoostPolygon(polyPoints1);
            BoostPolygon poly2 = MakeBoostPolygon(polyPoints2);
            std::cout << "poly1: " << bg::dsv(poly1) << std::endl;
            std::cout << "poly2:" << bg::dsv(poly2) << std::endl;
            std::vector<BoostPolygon> outPolys;
            bg::intersection(poly1, poly2, outPolys);
            std::cout << "*****intersection.****" << std::endl;

            double area = 0.0;
            for (const auto &poly : outPolys)
            {
                area += abs(bg::area(poly));
                std::cout << bg::dsv(poly) << std::endl;
            }
            std::cout << "*********" << std::endl;

            return area;
        }

        double DifferenceArea(std::vector<StorageType> polyPoints1,
                              std::vector<StorageType> polyPoints2)
        {
            if (polyPoints1.empty() || polyPoints2.empty())
            {
                return 0.0;
            }

            if (polyPoints1.front() != polyPoints1.back())
            {
                polyPoints1.push_back(polyPoints1.front());
            }

            if (polyPoints2.front() != polyPoints2.back())
            {
                polyPoints2.push_back(polyPoints2.front());
            }

            BoostPolygon poly1 = MakeBoostPolygon(polyPoints1);
            BoostPolygon poly2 = MakeBoostPolygon(polyPoints2);
            std::vector<BoostPolygon> outPolys;
            bg::difference(poly1, poly2, outPolys);
            double area = 0.0;
            for (const auto &poly : outPolys)
            {
                area += abs(bg::area(poly));
            }
            return area;
        }

        BoostPoint Center(const std::vector<BoostPoint> &points)
        {
            BoostPoint cPoint;
            COORDTYPE xCenter = 0.0;
            COORDTYPE yCenter = 0.0;
            for (const auto &point : points)
            {
                xCenter += point.get<0>();
                yCenter += point.get<1>();
            }
            cPoint = {xCenter / points.size(), yCenter / points.size()};
            return cPoint;
        }

        BoostPoint ClosestPoint(const std::vector<BoostPoint> &points, const BoostPoint &point)
        {
            BoostPoint closestPoint;
            COORDTYPE minDist = 1e10;
            for (const auto &otrPoint : points)
            {
                COORDTYPE dist = bg::distance(otrPoint, point);
                if (dist < minDist)
                {
                    minDist = dist;
                    closestPoint = otrPoint;
                }
            }
            return closestPoint;
        }

        BoostPoint ClosestPoint(const std::vector<BoostPoint> points, const BoostLine &line)
        {
            BoostPoint closestPoint;
            COORDTYPE minDist = 1e10;
            for (const auto &otrPoint : points)
            {
                COORDTYPE dist = bg::distance(otrPoint, line);
                if (dist < minDist)
                {
                    minDist = dist;
                    closestPoint = otrPoint;
                }
            }
            return closestPoint;
        }

        void PrintGeometry(const BoostPolygon &poly)
        {
            std::cout << bg::dsv(poly) << std::endl;
        }

        bool ToLeft(const BoostPoint &p, const BoostPoint &q, const BoostPoint &s)
        {
            COORDTYPE p_x = p.get<0>();
            COORDTYPE p_y = p.get<1>();
            COORDTYPE q_x = q.get<0>();
            COORDTYPE q_y = q.get<1>();
            COORDTYPE s_x = s.get<0>();
            COORDTYPE s_y = s.get<1>();
            COORDTYPE S = p_x * q_y - p_y * q_x + q_x * s_y - q_y * s_x + s_x * p_y - s_y * p_x;
            return S > 0;
        }

        bool ToLeft(const StorageType &p, const StorageType &q, const StorageType &s)
        {
            COORDTYPE p_x = p.x();
            COORDTYPE p_y = p.y();
            COORDTYPE q_x = q.x();
            COORDTYPE q_y = q.y();
            COORDTYPE s_x = s.x();
            COORDTYPE s_y = s.y();
            COORDTYPE S = p_x * q_y - p_y * q_x + q_x * s_y - q_y * s_x + s_x * p_y - s_y * p_x;
            return S > 0;
        }

        bool PointInPolygon(const StorageType &point, const std::vector<StorageType> &poly)
        {
            uint8_t leftPointCnt = 0U;
            uint8_t polyCnt = poly.size();
            for (int i = 1U; i < polyCnt + 1; ++i)
            {
                if (ToLeft(poly[i - 1], poly[i % polyCnt], point))
                {
                    ++leftPointCnt;
                }
            }
            return (leftPointCnt == polyCnt || leftPointCnt == 0U);
        }

        bool IsPolygonIntersect(const std::vector<StorageType> &poly1, const std::vector<StorageType> &poly2)
        {
            for (const auto &pt : poly1)
            {
                if (PointInPolygon(pt, poly2))
                {
                    return true;
                }
            }
            return false;
        }

        uint32_t GenHashCode(const BoostPoint point)
        {
            return uint32_t(point.get<0>() * 10000) + uint32_t(point.get<1>() * 10000) % 4294967295;
        }

        bool IsSegmentIntersect(std::pair<StorageType, StorageType> seg1,
                                std::pair<StorageType, StorageType> seg2)
        {
            StorageType p1 = seg1.first;
            StorageType p2 = seg1.second;

            StorageType q1 = seg2.first;
            StorageType q2 = seg2.second;

            bool c1 = ToLeft(p1, p2, q1);
            bool c2 = ToLeft(p1, p2, q2);

            bool c3 = ToLeft(q1, q2, p1);
            bool c4 = ToLeft(q1, q2, p2);

            return (c1 != c2) && (c3 != c4);
        }

        bool IsRectangleIntersect(std::array<StorageType, 4U> rect1, std::array<StorageType, 4U> rect2)
        {
            // 相交
            for (size_t i = 1U; i < 5U; ++i)
            {
                for (size_t j = 1U; j < 5U; ++j)
                {
                    auto seg1 = std::make_pair(rect1[i - 1], rect1[i % 4U]);
                    auto seg2 = std::make_pair(rect2[j - 1], rect2[j % 4U]);
                    if (IsSegmentIntersect(seg1, seg2))
                    {
                        return true;
                    }
                }
            }

            // 包含
            std::vector<StorageType> poly2{rect2.begin(), rect2.end()};
            for (size_t i = 0U; i < 4U; ++i)
            {
                if (PointInPolygon(rect1[i], poly2))
                {
                    return true;
                }
            }

            std::vector<StorageType> poly1{rect1.begin(), rect1.end()};
            for (size_t i = 0U; i < 4U; ++i)
            {
                if (PointInPolygon(rect2[i], poly1))
                {
                    return true;
                }
            }
            return false;
        }

        std::vector<uint32_t> GetNearbyElemByRTree(SpaceIndex::Ptr indexPtr, const StorageType &point, double bufferSize)
        {
            BoostMultiPolygon outPoly;
            BoostPoint bPoint = MakeBoostPoint(point);
            MakeBuffer(bPoint, outPoly, bufferSize);
            if (!outPoly.empty())
            {
                return indexPtr->GetIntersectElements(outPoly.front());
            }
            return {};
        }

        double PointToSegmentDist(const Eigen::Vector3d &p, const Eigen::Vector3d &q, const Eigen::Vector3d &s)
        {
            Eigen::Vector3d ps = p - s;
            Eigen::Vector3d qs = q - s;
            Eigen::Vector3d pq = p - q;
            double area = abs((ps.x() * qs.y() - ps.y() * qs.x()));
            double length = (q - p).norm();
            if (abs(length) < 1e-10)
            {
                return 0.0;
            }
            double height = area / length;

            if ((ps.dot(pq)) * (qs.dot(pq)) > 0)
            {
                return std::min(ps.norm(), qs.norm());
            }
            return height;
        }

    }
}
