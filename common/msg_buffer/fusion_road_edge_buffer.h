/*
 * @Author: guxiaojie
 * @Date: 2022-05-17 13:25:29
 * @LastEditTime: 2022-05-17 13:57:24
 * @LastEditors: guxiaojie
 * @Description: Do not edit
 * @FilePath: /lochdm/include/common/data_type/sensor/fusionroadedge.h
 */
#ifndef FUSION_FUSION_ROAD_EDGE_BUFFER_H
#define FUSION_FUSION_ROAD_EDGE_BUFFER_H

#include "utils/fixed_size_deque.h"
#include "inner_msg/fusion_road_edge.h"

namespace Fusion {

class FusionRoadedgeBuffer: public FixedSizeDeque<FusionRoadEdgeList::Ptr> {
public:
    static FusionRoadedgeBuffer& GetInstance() {
        static FusionRoadedgeBuffer instance(10U);
        return instance;
    }

private:
    FusionRoadedgeBuffer(const std::uint16_t fixedSize) : FixedSizeDeque<FusionRoadEdgeList::Ptr>(fixedSize) {}
    FusionRoadedgeBuffer(FusionRoadedgeBuffer&) = delete;
    FusionRoadedgeBuffer& operator= (const FusionRoadedgeBuffer&) = delete;
};

} // namespace Fusion

#endif