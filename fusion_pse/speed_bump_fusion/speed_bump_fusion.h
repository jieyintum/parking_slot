#ifndef __SPEED_BUMP_FUSION_H__
#define __SPEED_BUMP_FUSION_H__

#include <fstream>
#include <list>

#include "fusion_base.h"
#include "arrow_fusion/arrow_frame.h"
#include "utils/fixed_size_deque.h"
#include "inner_msgs/static_elem.h"

namespace Fusion {
namespace PSE {
class SpeedBumpFusion : public FusionBase<AvpeSpeedBumpFrame> {
public:
    using Ptr = std::shared_ptr<SpeedBumpFusion>;
    SpeedBumpFusion(rclcpp::Node& nh, FrameManager& frameManager) : FusionBase<AvpeSpeedBumpFrame>(nh, frameManager)
    {
#if DEBUG_MODE
        SpeedBumpMarkerPub_ = nh_.create_publisher<visualization_msgs::msg::MarkerArray>("/viz/pse/speed_bump_marker", 1);
#endif
    }
    ~SpeedBumpFusion() = default;

    void SetParkingMode(const std::atomic<bool>& mode);

private:
    bool MakeFrame(AvpeSpeedBumpFrame& frame) override;

    void Detect(AvpeSpeedBumpFrame& frame) override;

    void Track(AvpeSpeedBumpFrame& frame) override;

    void Output(AvpeSpeedBumpFrame& frame) override;

    void Debug(AvpeSpeedBumpFrame& frame) override;

    void Publish(AvpeSpeedBumpFrame& frame) override;

    void MergeSpeedBumps(const SpeedBumpTrack::Ptr& cur, const SpeedBumpTrack::Ptr& next);

    void AssociationRect(std::vector<SpeedBumpTrack::Ptr>& measure,
                         std::vector<int>& trackFlag,
                         std::vector<int>& measureFlag);

    void UpdateRect(const SpeedBumpTrack::Ptr& obj, const SpeedBumpTrack::Ptr& measure);

    void UpdateMeasureRect(const SpeedBumpTrack::Ptr& obj,const SpeedBumpTrack::Ptr& measure);

    void UpdateAttribute(const SpeedBumpTrack::Ptr& obj, const SpeedBumpTrack::Ptr& measure);
    
    double GetClosestPts(const SpeedBumpTrack::Ptr& a, const SpeedBumpTrack::Ptr& b, int& idx_a, int& idx_b);

    void DeleteUselessTrack(const Odometry& odom);

    void Reset();

    void GenerateNewId(const SpeedBumpTrack::Ptr& obj);

    void NumLimit(int& count);

#if DEBUG_MODE
    void ShowDetectObjs(const std::vector<SpeedBumpTrack::Ptr>& speedBumps, visualization_msgs::msg::MarkerArray& markers);
    void ShowTrackedObjs(const std::vector<SpeedBumpTrack::Ptr>& speedBumps, visualization_msgs::msg::MarkerArray& markers);
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr SpeedBumpMarkerPub_;
#endif


private:

    bool is_Parking_Mode_;
    Odometry lastOdom_;
    std::array<bool, 0x0f9f> idPool_ = {false};
    std::vector<SpeedBumpTrack::Ptr> trackObjs_;
    int max_track_size_ = 100;
    bool is_ego_static_ = true;
};
}
}
#endif // __SPEED_BUMP_FUSION_H__
