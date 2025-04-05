// Copyright (C) 2022 SAIC PP-CEM. All Rights Reserved.
// LEGAL NOTICE: All information contained herein is, and remains the property of SAIC PP-CEM.
#ifndef MAP_BUILDER_GEOM_UTILS_H
#define MAP_BUILDER_GEOM_UTILS_H
//  boost
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/foreach.hpp>

// eigne
#include <Eigen/Eigen>
#include <Eigen/Dense>

// std
#include <cmath>
#include <array>
#include <iostream>
#include <vector>
#include <deque>
#include <set>
#include <algorithm>
#include <time.h>

namespace Fusion {
namespace Geometry {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

#define DIMENSION 2U
#define COORD bg::cs::cartesian
#define COORDTYPE double


typedef bg::model::point<COORDTYPE, DIMENSION, COORD> BoostPoint;
typedef bg::model::multi_point<BoostPoint> BoostMultiPoint;
typedef bg::model::segment<BoostPoint> BoostSegment;
typedef bg::model::linestring<BoostPoint> BoostLine;
typedef bg::model::multi_linestring<BoostLine> BoostMultiLine;
typedef bg::model::ring<BoostPoint> BoostRing;
typedef bg::model::box<BoostPoint> BoostBox;
typedef bg::model::polygon<BoostPoint> BoostPolygon;
typedef bg::model::multi_polygon<BoostPolygon> BoostMultiPolygon;

typedef Eigen::Vector3d StorageType;

StorageType Point2d2Vector3d(const BoostPoint point);

BoostPoint MakeBoostPoint(COORDTYPE x, COORDTYPE y);

BoostPoint MakeBoostPoint(const StorageType& pointVector);

BoostMultiPoint MakeBoostMultiPoint(const std::vector<StorageType>& vectors);

BoostPolygon MakeBoostPolygon(const std::vector<StorageType>& vectors);

BoostPolygon MakeConvexHull(const BoostPolygon& poly);

BoostPolygon MakeConvexHull(const BoostMultiPoint& points);

void MakeConvexHull(const std::vector<StorageType>& vectors, std::vector<StorageType>& convexHull);

BoostBox MakeBoostBox(const std::pair<BoostPoint, BoostPoint> vectors);

BoostLine MakeBoostLine(const std::vector<StorageType>& vectors);

void MakeBuffer(const BoostPoint& point, BoostMultiPolygon& outPoly, 
                const COORDTYPE buffer_distance=1.0, const uint32_t points_per_circle=36);

void MakeBuffer(const BoostPolygon& inPoly, BoostMultiPolygon& outPoly, 
                const COORDTYPE buffer_distance=1.0, const uint32_t points_per_circle=36);

void MakeBuffer(const BoostLine& line, BoostMultiPolygon& outPoly, 
                const COORDTYPE buffer_distance=1.0, const uint32_t points_per_circle=36);

void BoostPolygonDiff(const BoostPolygon& p1, const BoostPolygon& p2, BoostMultiPolygon& outPoly);

bool IsCross(const BoostPoint& point, const BoostPolygon& poly);

bool IsCross(const BoostLine& line, const BoostPolygon& poly);

bool IsIntersects(const BoostPoint& point, const BoostPolygon& poly);

double IntersectArea(BoostPolygon poly1, BoostPolygon poly2);

double IntersectArea(const std::vector<Eigen::Vector3d> polyPoints1, 
                     const std::vector<Eigen::Vector3d> polyPoints2);

double DifferenceArea(const std::vector<StorageType> polyPoints1, 
                     const std::vector<StorageType> polyPoints2);

BoostPoint Center(const std::vector<BoostPoint>& points);

BoostPoint ClosestPoint(const std::vector<BoostPoint>& points, const BoostPoint& point);

BoostPoint ClosestPoint(const std::vector<BoostPoint> points, const BoostLine& line);

void PrintGeometry(const BoostPolygon& poly);

bool ToLeft(const BoostPoint& p, const BoostPoint& q, const BoostPoint& s);

/*
    Calc the point s location is left with p-q vector.
*/
bool ToLeft(const StorageType& p, const StorageType& q, const StorageType& s);

/*
    Judge is the point in the poylgon.
*/
bool PointInPolygon(const StorageType& point, const std::vector<StorageType>& poly);

bool IsPolygonIntersect(const std::vector<StorageType>& poly1, const std::vector<StorageType>& poly2);

bool IsSegmentIntersect(std::pair<Eigen::Vector3d, Eigen::Vector3d> seg1, 
                        std::pair<Eigen::Vector3d, Eigen::Vector3d> seg2);

bool IsRectangleIntersect(std::array<Eigen::Vector3d, 4U> rect1, std::array<Eigen::Vector3d, 4U> rect2);

uint32_t GenHashCode(BoostPoint& point);


// index type
typedef std::pair<BoostBox, uint32_t> idxValue;

class SpaceIndex {
public:

    SpaceIndex() 
    {
        rTree_ = {bgi::rtree<idxValue, bgi::rstar<16, 4>>()};
    }

    ~SpaceIndex() 
    {
        clear();
    }
    
    bool Insert(const BoostPoint& point, const uint32_t id)
    {
        BoostBox box {point, point};
        rTree_.insert(std::make_pair(box, id));
        return true;
    };

    using Ptr = std::shared_ptr<SpaceIndex>;

    bool Insert(const BoostPolygon& polygon, const uint32_t id)
    {
        BoostBox box = bg::return_envelope<BoostBox>(polygon);
        rTree_.insert(std::make_pair(box, id));
        return true;
    };

    bool Remove(const BoostPolygon& poly, const uint32_t id)
    {
        BoostBox box = bg::return_envelope<BoostBox>(poly);
        rTree_.remove(std::make_pair(box, id));
        return true;
    }

    std::vector<uint32_t> GetIntersectElements(BoostPolygon polygon) 
    {
        BoostBox box = bg::return_envelope<BoostBox>(polygon);
        std::vector<idxValue> queryResult;
        rTree_.query(bgi::intersects(box), std::back_inserter(queryResult));
        std::vector<uint32_t> queryIds;
        for(const auto& rst: queryResult) {
            queryIds.push_back(rst.second);
        }
        return queryIds;
    };

    std::vector<uint32_t> GetNearElements(BoostPoint point, COORDTYPE dist) 
    {
        COORDTYPE x = point.get<0U>();
        COORDTYPE y = point.get<1U>();
        BoostBox box(BoostPoint(x - dist, y - dist), BoostPoint(x + dist, y + dist));
        std::vector<idxValue> queryResult;
        rTree_.query(bgi::intersects(box), std::back_inserter(queryResult));
        std::vector<uint32_t> queryIds;
        for(const auto& rst: queryResult) {
            queryIds.push_back(rst.second);
        }
        return queryIds;
    };

    std::vector<uint32_t> GetNNElements(BoostPoint point, uint32_t n) 
    {
        std::vector<idxValue> queryResult;
        rTree_.query(bgi::nearest(point, n), std::back_inserter(queryResult));
        std::vector<uint32_t> queryIds;
        for(const auto& rst: queryResult) {
            queryIds.push_back(rst.second);
        }
        return queryIds;
    };

    std::vector<uint32_t> GetNNElements(BoostPolygon poly, uint32_t n) 
    {
        std::vector<idxValue> queryResult;
        rTree_.query(bgi::nearest(poly, n), std::back_inserter(queryResult));
        std::vector<uint32_t> queryIds;
        for(const auto& rst: queryResult) {
            queryIds.push_back(rst.second);
        }
        return queryIds;
    };

    uint32_t size() const 
    {
        return rTree_.size(); 
    }

    bool empty() const
    {
        return rTree_.empty();
    }

    void clear() 
    {
        rTree_.clear();
        rTree_ = {bgi::rtree<idxValue, bgi::rstar<16, 4>>()};
    };

private:
    bgi::rtree<idxValue, bgi::rstar<16, 4>> rTree_;

};

std::vector<uint32_t> GetNearbyElemByRTree(SpaceIndex::Ptr indexPtr, const StorageType& point, double bufferSize);

double PointToSegmentDist(const Eigen::Vector3d& p, const Eigen::Vector3d& q, const Eigen::Vector3d& s);

}
}

#endif

