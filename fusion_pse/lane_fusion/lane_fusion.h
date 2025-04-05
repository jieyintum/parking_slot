//
// Created by igs on 2022/12/5.
//

#pragma once

#include "memory"
#include "deque"
#include "lane_frame.h"
#include "utils/fixed_size_deque.h"
#include "fusion_base.h"
#include "lane_fusion/poly_line/poly_line_fitting.h"
#include "utils/kalman_filter.h"
#include "msg_buffer/odom_buffer.h"
#include "inner_msgs/static_elem.h"
#include "avpe_utils/avpe_grid_map.h"
#include "mpc_prk_msgs/msg/msg_sm_hmi2_pnc.hpp"
#include "vehicle_msgs/msg/can_copy_cp2_ap50ms_struct.hpp"

namespace Fusion {
namespace PSE {


class LaneFusion : public FusionBase<AvpeLaneFrame> {
public:
    using Ptr = std::shared_ptr<LaneFusion>;

    LaneFusion(rclcpp::Node& nh, FrameManager& frameManager) : FusionBase<AvpeLaneFrame>(nh, frameManager)
    {
        oneClusterSizeThre_ = 40; ///num
        distThre_ = 0.2; ///m

        fusionLaneMarkerPub_ = nh_.create_publisher<visualization_msgs::msg::MarkerArray>(
                "/debug/pse/lanes_track_marker", 1);
        detectPointsMarkerPub_ = nh_.create_publisher<visualization_msgs::msg::MarkerArray>(
                "/debug/pse/detect_points", 1);
        noMatchPointsMarkerPub_ = nh_.create_publisher<visualization_msgs::msg::MarkerArray>(
                "/debug/pse/no_match_points", 1);
        parkStateMaschine2PsfSub_ = nh_.create_subscription<mpc_prk_msgs::msg::MsgSmHmi2Pnc>("mpc_prk_sm2pnc", 10,
                                                                                             std::bind(
                                                                                                     & LaneFusion::DetectParkingStatus,
                                                                                                     this,
                                                                                                     std::placeholders::_1));
        canForDoorsStatusSub_ = nh_.create_subscription<vehicle_msgs::msg::CanCopyCp2Ap50msStruct>(
                "/mpc_cp2ap_50ms_CanCopyCp2Ap50msStruct", 10,
                std::bind(& LaneFusion::CANCallBack, this, std::placeholders::_1));
    }

    ~LaneFusion() = default;

private:

    /**
    * @brief (Virtual function)generate one original frame
    * @param input -> frame
    * @param output -> frame
    * @return true->success, false->failed
    */
    bool MakeFrame(AvpeLaneFrame& frame) override;

    /**
    * @brief (Virtual function)generate detect points
    * @param input -> frame
    * @param output -> frame
    * @return void
    */
    void Detect(AvpeLaneFrame& frame) override;

    /**
    * @brief generate one frame's detect points
    * @param input -> frame
    * @param output -> frame.lineFrame_
    * @return void
    */
    static void SaveOneFrame(AvpeLaneFrame& frame);

    /**
    * @brief generate multi frame's detect points
    * @param input -> frame
    * @param output -> multiFramesPoints_
    * @return void
    */
//    void SaveMultiFrame(AvpeLaneFrame& frame);

    /**
    * @brief DownSampling points
    * @param input -> frame
    * @param output -> frame.lineFrame_->detectClusters_
    * @return void
    */
//    void DownSampling(AvpeLaneFrame& frame);

    /**
    * @brief (Virtual function)track lines
    * @param input -> frame
   * @param output -> frame
    * @return void
    */
    void Track(AvpeLaneFrame& frame) override;

    /**
   * @brief match line with detect points, if not match, push detect points into noMatchPoints
   * @param input -> detectPoints
   * @param output -> noMatchPoints
   * @return void
   */
    void Match(const std::vector<Point::Ptr>& detectPoints,
               std::vector<Point::Ptr>& noMatchPoints);

    /**
   * @brief Use no match points to generate newParam, and use newParam to
     * Update track line's params, include start, end, k ,b
   * @param input -> newParam
   * @param output -> track
   * @return void
   */
    static void ParamUpdate(LineParam& newParam, FusionLane::Ptr& track);

    /**
   * @brief Accroding shrinkScale to shrink line
   * @param input -> trackLanes_
   * @param output -> trackLanes_
   * @return void
   */
    static void ParamPredict(FusionLane::Ptr& track);

    /**
   * @brief According shrinkScale to shrink line
   * @param input -> trackLanes_
   * @param output -> trackLanes_
   * @return void
   */
    void FusionFilter();

    /**
    * @brief if track line's life count == 0, clear untracked line
    * @param input -> trackLanes_
    * @param output -> trackLanes_
    * @return void
    */
    void Clear();

    /*void Match(const std::vector<ClusterFrame::Ptr>& clusters,
               std::vector<Point::Ptr>& noMatchPoints);*/
    /**
    * @brief Set grid map
    * @param input -> dataIn
    * @param output -> gridMapCluster_
    * @return void
    */
    void SetGridMap(const std::vector<Point::Ptr>& dataIn);

    /**
    * @brief According grid map to cluster points
    * @param input -> gridMapCluster_
    * @param output -> allClusters
    * @return void
    */
    void Cluster(std::vector<ClusterFrame::Ptr>& allClusters);

    /**
    * @brief In base coordinate to find each cluster's start point and end point
    * @param input -> frame.lineFrame_->noMatchPointsClusters_
    * @param output -> cluster->sEPointsBase
    * @return void
    */
    static void FindStartAndEndPoint(AvpeLaneFrame& frame);

    /**
    * @brief According A* to find each cluster's path from start point to end point
    * @param input -> frame.lineFrame_->noMatchPointsClusters_
    * @param output -> pathWorld
    * @return void
    */
    static void FindPath(AvpeLaneFrame& frame);

    /**
    * @brief Generate newLines_ according class DP
    * @param input -> frame.lineFrame_->noMatchPointsClusters_
    * @param output -> newLines_
    * @return void
    */
    void Generate(AvpeLaneFrame& frame);

    void GenerateNewId(FusionLane::Ptr& line);

    /**
    * @brief add new lines to trackLanes
    * @param input -> newLines_
    * @param output -> trackLanes_
    * @return void
    */
    void Add();

    /**
    * @brief if track line's life count > 6, output track lines
    * @param input -> frame
    * @return void
    */
    void Output(AvpeLaneFrame& frame) override;

    /**
    * @brief none
    * @param none
    * @return void
    */
    void Publish(AvpeLaneFrame& frame) override;

    void DetectParkingStatus(const mpc_prk_msgs::msg::MsgSmHmi2Pnc::SharedPtr msg);

    void CANCallBack(const vehicle_msgs::msg::CanCopyCp2Ap50msStruct::SharedPtr msg);

    ///--------------------------DEBUG INFO-------------------------------
    void Debug(AvpeLaneFrame& frame) override;

    void ShowOdomLine(const std::vector<FusionLane::Ptr>& outputs);

    static void PubFusionLane(const FusionLane::Ptr& fusionLane,
                              visualization_msgs::msg::MarkerArray& markers);

    void ShowdetectPoints(const LineFrame::Ptr& lineFrame);

    void ShowNoMatchPoints(const std::vector<Point::Ptr>& noMatchPoints);

    void ShowMatchPoints(const std::vector<Point::Ptr>& matchPoints);

    void PubTest(const FusionLane::Ptr& fusionLane,
                 visualization_msgs::msg::MarkerArray& markers);


private:
    // status
    bool isParking_ = false;
    bool isDoorStatusOk_ = false;
    rclcpp::Subscription<mpc_prk_msgs::msg::MsgSmHmi2Pnc>::SharedPtr parkStateMaschine2PsfSub_;
    rclcpp::Subscription<vehicle_msgs::msg::CanCopyCp2Ap50msStruct>::SharedPtr canForDoorsStatusSub_;

    //detect
    Odometry preOdom_;
    FixedSizeDeque<LineFrame::Ptr> multiFrames_{10};
    std::vector<Point::Ptr> multiFramesPoints_;
    int oneClusterSizeThre_;
    std::shared_ptr<AvpeGridMap> gridMapDownSampling_;

    //track
    double distThre_;
    std::shared_ptr<AvpeGridMap> gridMapCluster_;
    std::list<FusionLane::Ptr> trackLanes_;
    std::vector<FusionLane::Ptr> newLines_;
    std::array<bool, 0xffff> idPool_ = {false};

    //rviz
//    std::vector<Point::Ptr> noMatchPoints_;
//    std::vector<Point::Ptr> matchPoints_;

    //debug
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr noMatchPointsMarkerPub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fusionLaneMarkerPub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detectPointsMarkerPub_;
};
}
}

