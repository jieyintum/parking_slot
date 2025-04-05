#ifndef FUSION_FUSION_EDGE_LINE_H_
#define FUSION_FUSION_EDGE_LINE_H_

#include "cem_interfaces/msg/geometry_line_struct.hpp"

namespace Fusion {
struct FusionEdgeLine : LineBase {
    FusionEdgeLine(const cem_interfaces::msg::GeometryLineStruct& msg) :
        LineBase(msg.curve.c0, msg.curve.c1, msg.curve.c2, msg.curve.c3,
            msg.curve.longitudinal_distance_start, msg.curve.longitudinal_distance_end,
            msg.exist_probability),
        isValid(msg.is_curve_valid), measurementState(msg.measurement_state), lifeTime(msg.life_time),
        lastMeasureTimeStamp(msg.last_measured_time_stamp), isSamplePointValid(msg.is_sampling_point_valid)
    {
        samplePoints_.reserve(msg.sampling_point_list.size());
        for (const auto& ptIn : msg.sampling_point_list) {
            samplePoints_.emplace_back(ptIn.x, ptIn.y);
        }
    }

    bool isValid;
    uint8_t measurementState;
    float lifeTime;
    uint64_t lastMeasureTimeStamp;

    bool isSamplePointValid;    // 当前仅当 该位是valid时候，samplePoints_里面才有值
    std::vector<Eigen::Vector2d> samplePoints_;
};
}
#endif