//
// Created by igs on 2022/12/14.
//

#ifndef LANE_DATA_STRUCT_H
#define LANE_DATA_STRUCT_H

#include <utility>

#include "utils/time_stamp.h"
#include "utils/pld_utils.h"
#include "memory"
#include "inner_msgs/odometry.h"
#include "inner_msgs/avpe_segmentation.h"
#include "frame_base.h"
#include "utils/kalman_filter.h"
#include "inner_msgs/static_elem.h"
#include "avpe_utils/avpe_grid_map.h"
#include "utils/fixed_size_deque.h"

namespace Fusion {
namespace PSE {

struct LineParam {
    LineParam() = default;

    double start = 0;
    double end = 0;
    double k = 0;
    double b = 0;
    bool isInverse = false;

    double GetY(const double& x) const
    {
        return this->k * x + this->b;
    }

};

struct Point {
    using Ptr = std::shared_ptr<Point>;

    Point() = default;

    Eigen::Vector3d point;

    ///astar
    GridIndex index;
    int F = 0, G = 0, H = 0;
    Point::Ptr parent;

    ///DP
    int id = 0;
    bool isRemoved = false;
};

struct FusionLane {
    using Ptr = std::shared_ptr<FusionLane>;

    StaticElement ToMsg() const
    {
        std::vector<VertexStruct> vertexList{};

        VertexStruct vStart;
        vStart.x = this->param_.isInverse ? static_cast<float>(this->param_.GetY(this->param_.start))
                                          : static_cast<float>(this->param_.start);
        vStart.y = this->param_.isInverse ? static_cast<float>(this->param_.start)
                                          : static_cast<float>(this->param_.GetY(this->param_.start));

        vertexList.emplace_back(vStart);

        VertexStruct vEnd;
        vEnd.x = this->param_.isInverse ? static_cast<float>(this->param_.GetY(this->param_.end))
                                        : static_cast<float>(this->param_.end);
        vEnd.y = this->param_.isInverse ? static_cast<float>(this->param_.end)
                                        : static_cast<float>(this->param_.GetY(this->param_.end));
        vertexList.emplace_back(vEnd);

        StaticElement staticElem(vertexList,
                                 StaticElement::Classification::GROUND_LANE_LINE);

        return staticElem;
    }

    double GetDistance(const Point::Ptr& pt) const
    {
        if (param_.isInverse) {
            return std::abs(param_.k * pt->point.y() - pt->point.x() + param_.b) / (1.0 + (param_.k * param_.k));
        } else {
            return std::abs(param_.k * pt->point.x() - pt->point.y() + param_.b) / (1.0 + (param_.k * param_.k));
        }
    }

    LineParam param_;

    ///track
    std::vector<Point::Ptr> relativePoints;
    int lifeCount = 3;

    ///output
    uint16_t id = 5000;
};


struct ClusterFrame {
    using Ptr = std::shared_ptr<ClusterFrame>;

    std::vector<Point::Ptr> pointsWorld;
    std::vector<Point::Ptr> pointsBase;
    std::array<Point::Ptr, 2U> sEPointsWorld;
    std::array<Point::Ptr, 2U> sEPointsBase;

    ///A*
    std::vector<Point::Ptr> pathWorld;

    ///DP
    std::vector<FusionLane::Ptr> lines;

    void SetLines(const std::vector<LineParam>& paramsIn)
    {
        lines.clear();
        for (const auto& param : paramsIn) {
            FusionLane::Ptr line = std::make_shared<FusionLane>();
            line->param_ = param;
            lines.emplace_back(line);
        }
    }

    void SetIndex(const Eigen::Vector3d& pointsIn,
                  const GridIndex& indexIn)
    {
        Point::Ptr temp = std::make_shared<Point>();
        temp->point = pointsIn;
        temp->index = indexIn;
        this->pointsWorld.emplace_back(temp);
    }

    void TransToBase(const Odometry& odom)
    {
        for (const auto& point : pointsWorld) {
            Point::Ptr pt = std::make_shared<Point>();
            pt->point = (odom.orientation.inverse()) * (point->point - odom.translation);
            pt->index = point->index;
            pointsBase.emplace_back(pt);
        }
    }

    void TransToWorld(const Odometry& odom)
    {
        for (int i = 0 ; i < int(sEPointsBase.size()) ; ++i) {
            Point::Ptr temp = std::make_shared<Point>();
            temp->index = sEPointsBase[i]->index;
            temp->point = odom.orientation * sEPointsBase[i]->point + odom.translation;
            sEPointsWorld[i] = temp;
        }
    }

    void SetPath(const std::vector<Point::Ptr>& pathIn)
    {
        this->pathWorld = pathIn;
        for (auto& pt : this->pathWorld) {
            for (const auto& point : this->pointsWorld) {
                if (point->index.row == pt->index.row &&
                    point->index.col == pt->index.col) {
                    pt->point = point->point;
                }
            }
        }
    }


};

struct LineFrame {
    using Ptr = std::shared_ptr<LineFrame>;

//    std::vector<ClusterFrame::Ptr> detectClusters_;
    std::vector<ClusterFrame::Ptr> noMatchPointsClusters_;
    std::vector<Point::Ptr> detectPointsBase_;
    std::vector<Point::Ptr> detectPointsWorld_;

    void SetPointsBase(std::vector<SegPoint>& pointsIn)
    {
        for (const auto& point : pointsIn) {
            Point::Ptr temp = std::make_shared<Point>();
            temp->point.x() = point.point.x();
            temp->point.y() = point.point.y();
            temp->point.z() = 0.0;
            detectPointsBase_.emplace_back(temp);
        }
    }

    void TransToWorld(const Odometry& odom)
    {
        detectPointsWorld_.clear();
        for (const auto& point : detectPointsBase_) {
            Point::Ptr pt = std::make_shared<Point>();
            pt->point = odom.orientation * point->point + odom.translation;
            detectPointsWorld_.emplace_back(pt);
        }
    }

};

struct AvpeLaneFrame : public FrameBase {
    ///input
    AvpeSegArray::Ptr lane_;
    ///process
    LineFrame::Ptr lineFrame_;
    ///output
    std::vector<FusionLane::Ptr> outputLanes_;
};

}
}

#endif //LANE_DATA_STRUCT_H
