#ifndef __ZEBRA_FUSION_H__
#define __ZEBRA_FUSION_H__

#include <list>

#include "fusion_base.h"
#include "utils/geom_utils.h"
#include "avpe_utils/avpe_grid_map.h"
#include "zebra_fusion/zebra_frame.h"
#include "utils/fixed_size_deque.h"
#include "mpc_prk_msgs/msg/msg_sm_hmi2_pnc.hpp"
#include "vehicle_msgs/msg/can_copy_cp2_ap50ms_struct.hpp"

namespace Fusion {
namespace PSE {
class ZebraFusion : public FusionBase<AvpeZebraFrame> {
public:
    using Ptr = std::shared_ptr<ZebraFusion>;

    ZebraFusion(rclcpp::Node& nh, FrameManager& frameManager) : FusionBase<AvpeZebraFrame>(nh, frameManager)
    {
        zebraId_ = 4501;

        gMapPtr_ = std::make_shared<AvpeGridMap>(0., 0., this->mapRange_, this->mapRange_, this->mapPercesion_);

        fusionOdomZebraMarkerPub_ = nh_.create_publisher<visualization_msgs::msg::MarkerArray>(
                "/debug/fusion_zebra_odom_marker", 1);
        /* fusionPerceptionZebraMarkerPub_ = nh_.create_publisher<visualization_msgs::msg::MarkerArray>(
                 "/debug/fusion_zebra_per_marker", 1);*/
        zebraPointsMarkerPub_ = nh_.create_publisher<visualization_msgs::msg::Marker>(
                "/debug/zebra_origin_points", 1);
        textureMarkerPub_ = nh_.create_publisher<visualization_msgs::msg::MarkerArray>(
                "/debug/zebra_texture_box", 1);
        rtreeIndexPtr = std::make_shared<Geometry::SpaceIndex>();
        parkStateMaschine2PsfSub_ = nh_.create_subscription<mpc_prk_msgs::msg::MsgSmHmi2Pnc>("mpc_prk_sm2pnc", 10,
                                                                                             std::bind(
                                                                                                     & ZebraFusion::DetectParkingStatus,
                                                                                                     this,
                                                                                                     std::placeholders::_1));
        canForDoorsStatusSub_ = nh_.create_subscription<vehicle_msgs::msg::CanCopyCp2Ap50msStruct>(
                "/mpc_cp2ap_50ms_CanCopyCp2Ap50msStruct", 10,
                std::bind(& ZebraFusion::CANCallBack, this, std::placeholders::_1));

    }

    ~ZebraFusion() = default;

private:
    bool MakeFrame(AvpeZebraFrame& frame) override;

    void Detect(AvpeZebraFrame& frame) override;

    void Track(AvpeZebraFrame& frame) override;

    void Output(AvpeZebraFrame& frame) override;

    void Debug(AvpeZebraFrame& frame) override;

    void Publish(AvpeZebraFrame& frame) override;

    void StoreZebraPoint(AvpeZebraFrame& frame);

    void InitLocalMap(AvpeZebraFrame& frame);

    bool CreateGridMap(AvpeGridMap::Ptr& gMapPtr);

    void Cluster(AvpeZebraFrame& frame);

    void ZebraTextureCluster(AvpeZebraFrame& frame);

    void ZebraCluster(AvpeZebraFrame& frame);

    double GetZebraTextureDist(const FusionZebra::Ptr& fusionZebraPtr1, const FusionZebra::Ptr& fusionZebraPtr2);

    double GetZebraTextureDirInnerProduct(const FusionZebra::Ptr& fusionZebraPtr1,
                                          const FusionZebra::Ptr& fusionZebraPtr2);

    void InsertZebraTexutres(const AvpeZebraFrame& frame);

    void ZebraClusterSingle(const std::vector<FusionZebra::Ptr>& zebras, const uint32_t curId,
                            std::set<uint32_t>& closeIds, std::set<uint32_t>& openIds);

    void StoreMultiFrameZebra(AvpeZebraFrame& frame);

    bool IsMatched(const FusionZebra::Ptr& trackArrPtr, const FusionZebra::Ptr& fusionArrPtr);

    void MatchZebras(AvpeZebraFrame& frame);

    void AddNewFoundZebras(AvpeZebraFrame& frame);

    void ClearUntrackObject(AvpeZebraFrame& frame);

    void FusionMultiObjectZebra(FusionZebra::Ptr& fusionArrPtr);

    void UpdateTrackZebras();

    void FusionTrackedObjects();

    void CalcOneDimNormProbParam(const std::vector<double>& xs, double& mean, double& stdVar);

    void FusionByNormProbDistibute(const FusionZebra::Ptr& fusionArrPtr, std::vector<FusionZebra::Ptr>& fusionZebras);

    double GetTextureMeanLength(const FusionZebra::Ptr& fusionZebraPtr);

    void SortByZebraWidthError(std::vector<FusionZebra::Ptr>& fusionZebras, double zebraMeanWidth);

    void SortByZebraLength(std::vector<FusionZebra::Ptr>& fusionZebras, double topNRadio);

    void GetFusionedZebra(const FusionZebra::Ptr& fusionZebraPtr, FusionZebra::Ptr& fusionedZebraPtr);

    void UpdateTrackZebraPoints(FusionZebra::Ptr& trackZebraPtr, const FusionZebra::Ptr& fusionedZebraPtr);

    void RandomDownSampling();

//    void PointToMat();

    void DetectParkingStatus(const mpc_prk_msgs::msg::MsgSmHmi2Pnc::SharedPtr msg);

    void CANCallBack(const vehicle_msgs::msg::CanCopyCp2Ap50msStruct::SharedPtr msg);

#if DEBUG_MODE

    void ShowTextureBox(const std::vector<FusionZebra::Ptr>& zebras);

    void Pub(const FusionZebra::Ptr& fusionZebra,
             const uint16_t id,
             visualization_msgs::msg::MarkerArray& markers);

    void ShowPoint(AvpeZebraFrame& frame);

    void ShowOdomZebra(const std::vector<FusionZebra::Ptr>& zebras);

    void PushSingleOdomZebraMarker(const FusionZebra::Ptr& fusionZebra,
            /*const uint16_t id,*/
                                   visualization_msgs::msg::MarkerArray& markers);

    void PushSingleOdomZebraBoxMarker(const FusionZebra::Ptr& fusionZebra,
            /*const uint16_t id,*/
                                      visualization_msgs::msg::MarkerArray& markers);

    void ShowPerceptionZebra(const FusionZebra::Ptr& zebras);

    void PushSinglePerceptionZebraMarker(const FusionZebra::Ptr& fusionZebra, const uint16_t id,
                                         visualization_msgs::msg::MarkerArray& markers);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fusionOdomZebraMarkerPub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr zebraPointsMarkerPub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr textureMarkerPub_;

#endif
    rclcpp::Subscription<mpc_prk_msgs::msg::MsgSmHmi2Pnc>::SharedPtr parkStateMaschine2PsfSub_;
    rclcpp::Subscription<vehicle_msgs::msg::CanCopyCp2Ap50msStruct>::SharedPtr canForDoorsStatusSub_;
private:
    // status
    bool isParking_ = false;
    bool isDoorStatusOk_ = false;

    int zebraId_;
    Odometry preOdom_;
    std::list<FusionZebra::Ptr> trackZebras_;
    Geometry::SpaceIndex::Ptr rtreeIndexPtr;
    FixedSizeDeque<FusionZebra::Ptr> multiFrameZebras_{5U};
    std::vector<Eigen::Vector3d> afterDownSampling_;
    // local grid map
    AvpeGridMap::Ptr gMapPtr_ = nullptr;

    // local map param
    const double mapPercesion_ = 0.05; // m
    const double mapRange_ = 10.0; // m
};

}
}

#endif // __ZEBRA_FUSION_H__