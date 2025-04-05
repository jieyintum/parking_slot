#ifndef FUSION_LINE_OBJECTS_H_
#define FUSION_LINE_OBJECTS_H_

#include <memory>
#include "cem_interfaces/msg/road_edge_struct.hpp"

#include "msg_base.h"
#include "line_base.h"
#include "inner_msg/line_object/line_position_type.h"

namespace Fusion {
struct LaneLineObj : LineBase {
    using Ptr = std::shared_ptr<LaneLineObj>;
    LaneLineObj(const f100c_msgs::msg::APF100CLane& msg, const LinePositionType& p) 
        : LineBase(msg.c0, msg.c1, msg.c2, msg.c3, msg.view_range_start, msg.view_range_end, msg.exist_prob),
          color(msg.marker_color), type(msg.marker_type), width(msg.line_width),
          position(p) {}

    LaneLineObj(const f100c_msgs::msg::APF100CRoadeage& msg, const LinePositionType& p)
        : LineBase(msg.c0, msg.c1, msg.c2, msg.c3, msg.view_range_start, msg.view_range_end, msg.exist_prob),
          color(0),
          type{msg.marker_type},
          width(0.20),
          position(p) {}

    LaneLineObj(const vision_msgs::msg::Bevlane& msg)
        : color(msg.type), type(msg.type), width(0.15), position(static_cast<LinePositionType>(msg.position))
    {
        std::vector<Eigen::Vector2d> points;
        points.reserve(msg.line_points.size());
        this->start_ = 1000.0;
        this->end_ = -1000.0;
        for (const auto& pt : msg.line_points) {
            points.emplace_back(pt.x, pt.y);
            this->start_ = std::min(this->start_, pt.x);
            this->end_ = std::max(this->end_, pt.x);
        }
        this->Polyfit(points);
        this->prob_ = msg.conf;
    }

    std::uint8_t color;
    std::uint8_t type;
    float width;
    LinePositionType position;
};

struct PerLineObjects : MsgBase {
public:
    using Ptr = std::shared_ptr<PerLineObjects>;
    PerLineObjects(const f100c_msgs::msg::APF100CData::SharedPtr& msg) : MsgBase(msg->header.stamp)
    {
        std::unordered_map<size_t, LinePositionType> positMap = {
            {0, LinePositionType::FIRST_ON_THE_LEFT},
            {1, LinePositionType::FIRST_ON_THE_RIGHT},
            {2, LinePositionType::SECOND_ON_THE_LEFT},
            {3, LinePositionType::SECOND_ON_THE_RIGHT}};
        for (size_t i = 0U; i < 4U; ++i) {
            laneLines[positMap[i]] = std::make_shared<LaneLineObj>(msg->lanes[i], positMap[i]);
            laneLines[positMap[i]]->c0_ = laneLines[positMap[i]]->c0_ < 0.0 ? laneLines[positMap[i]]->c0_ - 0.075 : laneLines[positMap[i]]->c0_ + 0.075;
        }

        for (size_t i = 0U; i < 2U; ++i) {
            roadEdges[positMap[i]] = std::make_shared<LaneLineObj>(msg->roadeages[i], positMap[i]);
        }
    }

    PerLineObjects(const vision_msgs::msg::Bevlanes::SharedPtr& msg) : MsgBase(msg->header.stamp)
    {
        for (const auto& msgLane : msg->lanes) {
            laneLines[static_cast<LinePositionType>(msgLane.position)] = std::make_shared<LaneLineObj>(msgLane);
        }
        for (const auto& msgEdge : msg->edges) {
            roadEdges[static_cast<LinePositionType>(msgEdge.position)] = std::make_shared<LaneLineObj>(msgEdge);
        }
    }
public:
    std::vector<LaneLineObj::Ptr> GetHostLaneLines()
    {
        std::vector<LaneLineObj::Ptr> hostLines;
        auto leftIter = laneLines.find(LinePositionType::FIRST_ON_THE_LEFT);
        if (leftIter != laneLines.end()) {
            hostLines.push_back(leftIter->second); 
        } else {
            hostLines.push_back(nullptr);
        }
        auto rightIter = laneLines.find(LinePositionType::FIRST_ON_THE_RIGHT);
        if (rightIter != laneLines.end()) {
            hostLines.push_back(rightIter->second);
        } else {
            hostLines.push_back(nullptr);
        }

        return hostLines;
    }

    std::vector<LaneLineObj::Ptr> GetMainFourLines()
    {
        std::vector<LaneLineObj::Ptr> mainLines = GetHostLaneLines();
        auto leftIter = laneLines.find(LinePositionType::SECOND_ON_THE_LEFT);
        if (leftIter != laneLines.end()) {
            mainLines.push_back(leftIter->second); 
        } else {
            mainLines.push_back(nullptr);
        }
        auto rightIter = laneLines.find(LinePositionType::SECOND_ON_THE_RIGHT);
        if (rightIter != laneLines.end()) {
            mainLines.push_back(rightIter->second);
        } else {
            mainLines.push_back(nullptr);
        }

        return mainLines;
    }

    std::vector<LaneLineObj::Ptr> GetLeftRoadEdges()
    {
        std::vector<LaneLineObj::Ptr> leftRoadEdges;
        for (const auto& edge : roadEdges) {
            if ((edge.first > LinePositionType::UNKNOW) && (edge.first < LinePositionType::FIRST_ON_THE_RIGHT)) {
                leftRoadEdges.push_back(edge.second);
            }
        }

        return leftRoadEdges;
    }

    std::vector<LaneLineObj::Ptr> GetRightRoadEdges()
    {
        std::vector<LaneLineObj::Ptr> rightRoadEdges;
        for (const auto& edge : roadEdges) {
            if ((edge.first > LinePositionType::SEVENTH_ON_THE_LEFT) && (edge.first < LinePositionType::OTHER)) {
                rightRoadEdges.push_back(edge.second);
            }
        }

        return rightRoadEdges;
    }
public:
    std::map<LinePositionType, LaneLineObj::Ptr> laneLines;
    std::map<LinePositionType, LaneLineObj::Ptr> roadEdges;
};
}   // namespace Fusion
#endif