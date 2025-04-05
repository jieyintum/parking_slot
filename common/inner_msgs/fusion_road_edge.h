#ifndef FUSION_FUSION_ROAD_EDGE_H_
#define FUSION_FUSION_ROAD_EDGE_H_

#include "cem_interfaces/msg/traffic_road_edge_struct.hpp"
#include "cem_interfaces/msg/road_edge_struct.hpp"

#include "inner_msg/line_object/fusion_edge_line.h"

namespace Fusion {

struct FusionRoadEdgeSegment : FusionEdgeLine {
    using Ptr = std::shared_ptr<FusionRoadEdgeSegment>;
    FusionRoadEdgeSegment(const cem_interfaces::msg::SegmentRoadEdgeElementStruct& msg)
        : FusionEdgeLine(msg.geometry), id (msg.id), sensors(msg.contributing_sensors), type(msg.road_edge_type),
          height(msg.height_above_ground) {}

    uint32_t id;
    uint16_t sensors;
    uint8_t type;
    float height;
};

struct FusionRoadEdge {
    using Ptr = std::shared_ptr<FusionRoadEdge>;

    FusionRoadEdge(const cem_interfaces::msg::RoadEdgeStruct& msg)
        : isValid(!msg.segment_road_edge_elements_list.empty()), id(msg.id), position(msg.position)
    {
        segmentIds.reserve(msg.linked_lane_segment_id_list.size());
        for (const auto id : msg.linked_lane_segment_id_list) {
            segmentIds.push_back(id);
        }

        segments.reserve(msg.segment_road_edge_elements_list.size());
        for (const auto& segmentIn : msg.segment_road_edge_elements_list) {
            segments.emplace_back(std::make_shared<FusionRoadEdgeSegment>(segmentIn));
        }
    }

    bool isValid;
    uint32_t id;
    uint8_t position;   // 0 - unknown  1 - left  2 - right   3 - others
    std::vector<uint32_t> segmentIds;
    std::vector<FusionRoadEdgeSegment::Ptr> segments;
};

class FusionRoadEdgeList : public MsgBase {
public:
    FusionRoadEdgeList() = default;
    ~FusionRoadEdgeList() = default;
    using Ptr = std::shared_ptr<FusionRoadEdgeList>;

    FusionRoadEdgeList(const cem_interfaces::msg::TrafficRoadEdgeStruct::SharedPtr& msg):
        MsgBase(TimeStamp(msg->header.stamp))
    {
        for (size_t i = 0U; i < msg->road_edges_list.size(); ++i) {
            const auto& roadEdge = msg->road_edges_list[i];
            if (i == 0U) {
                roadEdges[LinePositionType::FIRST_ON_THE_LEFT] = std::make_shared<FusionRoadEdge>(roadEdge);
            } else if (i == 1U) {
                roadEdges[LinePositionType::FIRST_ON_THE_RIGHT] = std::make_shared<FusionRoadEdge>(roadEdge);
            } else {
                if (roadEdge.position != 1U && roadEdge.position != 2U) {
                    continue;
                }
                const auto positionType = GenerateMapIndex(roadEdge.position);
                if (positionType != LinePositionType::UNKNOW) {
                    roadEdges[positionType] = std::make_shared<FusionRoadEdge>(roadEdge);
                } else {
                    std::cout << "Warning: Cannot Get Fusion Edge's Position Type" << std::endl;
                }
            }
        }
    }

    std::map<LinePositionType, FusionRoadEdge::Ptr> roadEdges;

private:
    LinePositionType GenerateMapIndex(const uint8_t position)
    {
        if (position == 1U) {   // left
            for (uint8_t i = static_cast<uint8_t>(LinePositionType::FIRST_ON_THE_LEFT);
                i <= static_cast<uint8_t>(LinePositionType::SEVENTH_ON_THE_LEFT); ++i) {
                if (this->roadEdges.find(static_cast<LinePositionType>(i)) == this->roadEdges.end()) {
                    return static_cast<LinePositionType>(i);
                }
            }
        } else {    // right
            for (uint8_t i = static_cast<uint8_t>(LinePositionType::FIRST_ON_THE_RIGHT);
                i <= static_cast<uint8_t>(LinePositionType::SEVENTH_ON_THE_RIGHT); ++i) {
                if (this->roadEdges.find(static_cast<LinePositionType>(i)) == this->roadEdges.end()) {
                    return static_cast<LinePositionType>(i);
                }
            }
        }

        return LinePositionType::UNKNOW;
    }
};
}
#endif