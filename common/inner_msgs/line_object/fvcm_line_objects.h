#ifndef FUSION_FVCM_LINE_H_
#define FUSION_FVCM_LINE_H_

#include <array>
#include "f100c_msgs/msg/apf100_c_data.hpp"
#include "time_stamp.h"
#include "line_base.h"

namespace Fusion {
struct FvcmLaneEdge : LineBase {
    FvcmLaneEdge() = default;
    FvcmLaneEdge(const f100c_msgs::msg::APF100CLane& msg) 
        : LineBase(msg.c0, msg.c1, msg.c2, msg.c3, msg.view_range_start, msg.view_range_end, msg.exist_prob),
          markerColor(msg.marker_color), markerType(msg.marker_type), lineWidth(msg.line_width)
    {
        isValid = ((prob_ > 0.5) && ((end_ - start_) > 10.0));
    }
    ~FvcmLaneEdge() = default;

    std::uint8_t markerColor;
    std::uint8_t markerType;
    float lineWidth;
    bool isValid;
};

struct FvcmRoadEdge : LineBase {
    FvcmRoadEdge() = default;
    FvcmRoadEdge(const f100c_msgs::msg::APF100CRoadeage& msg)
        : LineBase(msg.c0, msg.c1, msg.c2, msg.c3, msg.view_range_start, msg.view_range_end, msg.exist_prob),
          markerType{msg.marker_type}
    {
        isValid = prob_ > 0.9;
    }
    ~FvcmRoadEdge() = default;

    std::uint8_t markerType;
    bool isValid;
};


using FvcmDataMsg = f100c_msgs::msg::APF100CData;
using FvcmDataMsgPtr = FvcmDataMsg::SharedPtr;
struct FvcmLineObjects {
    using Ptr = std::shared_ptr<FvcmLineObjects>;
    FvcmLineObjects(const FvcmDataMsgPtr& msg) : timestamp(msg->header.stamp)
    {
        isFvcmLaneValid_ = (msg->lanes.size() == laneEdges.size());
        if (isFvcmLaneValid_) {
            for (size_t i = 0U; i < laneEdges.size(); ++i) {
                laneEdges[i] = FvcmLaneEdge(msg->lanes[i]);
                laneEdges[i].c0_ = laneEdges[i].c0_ > 0.0 ? laneEdges[i].c0_ + 0.075 : laneEdges[i].c0_ - 0.075;
            }
        }
        isFvcmLaneValid_ &= (laneEdges[LEFT].isValid && laneEdges[LEFT].c0_ > 0.0) || (laneEdges[RIGHT].isValid && laneEdges[RIGHT].c0_ < 0.0);

        isFvcmRoadedgeValid_ = (msg->roadeages.size() == roadEdges.size());
        if (isFvcmRoadedgeValid_) {
            for (size_t i = 0U; i < roadEdges.size(); ++i) {
                roadEdges[i] = FvcmRoadEdge(msg->roadeages[i]);
            }
        }
    }
    ~FvcmLineObjects() = default;

public:
    TimeStamp timestamp;
    std::array<FvcmLaneEdge, 4U> laneEdges;
    std::array<FvcmRoadEdge, 2U> roadEdges;
    bool isFvcmLaneValid_ ;
    bool isFvcmRoadedgeValid_;
private:
    enum RelativePosition {
        LEFT = 0,
        RIGHT = 1,
        LEFT_LEFT = 2,
        RIGHT_RIGHT = 3
    };
};

// class FvcmDataBuffer: public TimeFixedSizeDeque<FvcmLineObjects::Ptr>
// {
// public:
//     FvcmDataBuffer(const std::uint16_t bufferSize) : TimeFixedSizeDeque<FvcmLineObjects::Ptr>(bufferSize) {}
//     ~FvcmDataBuffer() = default;
//     using Ptr = std::shared_ptr<FvcmDataBuffer>;
// };

} // Fusion

#endif