#ifndef FUSION_BEV_LINE_OBJECT_H_
#define FUSION_BEV_LINE_OBJECT_H_

#include <memory>
#include "vision_msgs/msg/bevlanes.hpp"
#include "vision_msgs/msg/bevlane.hpp"
#include "time_stamp.h"
#include "line_base.h"


namespace Fusion {


struct BevLine : LineBase {
    enum class LinePositionType {
        UNKNOW               =0,
        FIRST_ON_THE_LEFT    =1,
        SECOND_ON_THE_LEFT   =2,
        THIRD_ON_THE_LEFT    =3,
        FORTH_ON_THE_LEFT    =4,
        FIFTH_ON_THE_LEFT    =5,
        SIXTH_ON_THE_LEFT    =6,
        SEVENTH_ON_THE_LEFT  =7,
        FIRST_ON_THE_RIGHT   =8,
        SECOND_ON_THE_RIGHT  =9,
        THIRD_ON_THE_RIGHT   =10,
        FORTH_ON_THE_RIGHT   =11,
        FIFTH_ON_THE_RIGHT   =12,
        SIXTH_ON_THE_RIGHT   =13,
        SEVENTH_ON_THE_RIGHT =14,
        OTHER                =15
    };
    BevLine(const vision_msgs::msg::Bevlane& msg)
        : id(msg.id), pointsNum(msg.number_of_points), position(static_cast<LinePositionType>(msg.position)),
          type(msg.type), color(msg.type)
    {
        points.reserve(pointsNum);
        for (auto& pt : msg.line_points) {
            points.emplace_back(pt.x, pt.y);
            this->start_ = std::min(this->start_, pt.x);
            this->end_ = std::max(this->end_, pt.y);
        }
        Polyfit(this->points);
        this->prob_ = msg.conf;
    }
    ~BevLine() = default;

    uint32_t id;
    std::uint8_t pointsNum;
    std::vector<Eigen::Vector2d> points;
    LinePositionType position;
    std::uint8_t type;
    std::uint8_t color;
};

using BevLanesMsg = vision_msgs::msg::Bevlanes;
using BevLanesMsgPtr = BevLanesMsg::SharedPtr;
struct BevLineObjects {
    using Ptr = std::shared_ptr<BevLineObjects>;
    BevLineObjects(const BevLanesMsgPtr& msg)
        : timestamp(msg->header.stamp), frameId(msg->frameid),
          laneNumber(msg->lane_num), edgeNumber(msg->edge_num)
    {
        for (const auto& msgLane : msg->lanes) {
            laneLines.emplace_back(msgLane);
        }
        for (const auto& msgEdge : msg->edges) {
            roadEdges.emplace_back(msgEdge);
        }
    }

    TimeStamp timestamp;
    int32_t frameId;
    std::uint16_t laneNumber;
    std::uint16_t edgeNumber;
    std::vector<BevLine> laneLines;
    std::vector<BevLine> roadEdges;
};
}

#endif