#ifndef ARROW_FUSION
#define ARROW_FUSION

#include <fstream>
#include <list>

#include "fusion_base.h"
#include "avpe_utils/avpe_grid_map.h"
#include "arrow_fusion/arrow_frame.h"
#include "utils/fixed_size_deque.h"
#include "inner_msgs/static_elem.h"
#include "mpc_prk_msgs/msg/msg_sm_hmi2_pnc.hpp"
#include "vehicle_msgs/msg/can_copy_cp2_ap50ms_struct.hpp"

namespace Fusion
{
    namespace PSE
    {
        class ArrowFusion : public FusionBase<AvpeArrowFrame>
        {
        public:
            using Ptr = std::shared_ptr<ArrowFusion>;
            ArrowFusion(rclcpp::Node &nh, FrameManager &frameManager) : FusionBase<AvpeArrowFrame>(nh, frameManager)
            {
                arrowId_ = 4001;
                fusionOdomArrowMarkerPub_ = nh_.create_publisher<visualization_msgs::msg::MarkerArray>("/debug/fusion_arrow_odom_marker", 1);
                fusionPerceptionArrowMarkerPub_ = nh_.create_publisher<visualization_msgs::msg::MarkerArray>("/debug/fusion_arrow_per_marker", 1);
                parkStateMaschine2PsfSub_ = nh_.create_subscription<mpc_prk_msgs::msg::MsgSmHmi2Pnc>("mpc_prk_sm2pnc", 10, std::bind(&ArrowFusion::DetectParkingStatus, this, std::placeholders::_1));
                canForDoorsStatusSub_ = nh_.create_subscription<vehicle_msgs::msg::CanCopyCp2Ap50msStruct>("/mpc_cp2ap_50ms_CanCopyCp2Ap50msStruct", 10, std::bind(&ArrowFusion::CANCallBack, this, std::placeholders::_1));
                gMapPtr_ = std::make_shared<AvpeGridMapImg>(0., 0., this->mapRange_, this->mapRange_, this->mapPercesion_);
            }
            ~ArrowFusion() = default;

            void SetCurFrameLanLine(std::vector<StaticElement> &laneLines);

        private:
            bool MakeFrame(AvpeArrowFrame &frame) override;

            void Detect(AvpeArrowFrame &frame) override;

            void Track(AvpeArrowFrame &frame) override;

            void Output(AvpeArrowFrame &frame) override;

            void Debug(AvpeArrowFrame &frame) override;

            void Publish(AvpeArrowFrame &frame) override;

            void InitLocalMap(AvpeArrowFrame &frame);

            void StoreArrowPoint(AvpeArrowFrame &frame);

            bool CreateGridMap(AvpeGridMap::Ptr &gMapPtr);

            void ClusterArrowPoint(AvpeArrowFrame &frame);

            void StoreMultiFrameArrow(AvpeArrowFrame &frame);

            bool IsMatched(const FusionArrow::Ptr &trackArrowPtr, const FusionArrow::Ptr &fusionArrowPtr);

            void MatchArrows(AvpeArrowFrame &frame);

            void AddNewFoundArrows(AvpeArrowFrame &frame);

            void ClearUntrackObject(AvpeArrowFrame &frame);

            void FusionMultiObjectArrow(FusionArrow::Ptr &fusionArrowPtr);

            void UpdateTrackArrows();

            void FusionTrackedObjects();

            void CalcOneDimNormProbParam(const std::vector<double> &xs, double &mean, double &stdVar);

            void FusionByNormProbDistibute(const FusionArrow::Ptr &fusionArrowPtr, std::vector<FusionArrow::Ptr> &fusionArrows);

            int GetSubType(std::vector<int> types);

            Eigen::Vector3d GenRightEndPoint(const double width, const Eigen::Vector3d &left,
                                             const Eigen::Vector3d &right, const Eigen::Vector3d &basePoint);

            Eigen::Vector3d GenLeftEndPoint(const double width, const Eigen::Vector3d &left,
                                            const Eigen::Vector3d &right, const Eigen::Vector3d &basePoint);

            void GenVirtualBox();

            void DetectParkingStatus(const mpc_prk_msgs::msg::MsgSmHmi2Pnc::SharedPtr msg);

            void GenArrowDir(AvpeArrowFrame &frame);

            void GenComplexArrowDir(FusionArrow::Ptr &trackArrow);

            void GenSimpleArrowDir(FusionArrow::Ptr &trackArrow);

            void CorrectArrowDir(AvpeArrowFrame &frame, FusionArrow::Ptr &trackArrow);
            void CANCallBack(const vehicle_msgs::msg::CanCopyCp2Ap50msStruct::SharedPtr msg);

#if DEBUG_MODE

            void ShowOdomArrow(const std::vector<FusionArrow::Ptr> &arrows);
            void PushSingleOdomArrowMarker(const FusionArrow::Ptr &fusionArrow, const uint16_t id,
                                           visualization_msgs::msg::MarkerArray &markers);
            void PushSingleOdomArrowBoxMarker(const FusionArrow::Ptr &fusionArrow, const uint16_t id,
                                              visualization_msgs::msg::MarkerArray &markers);
            void PushSingleOdomArrowPolyMarker(const FusionArrow::Ptr &fusionArrow, const uint16_t id,
                                               visualization_msgs::msg::MarkerArray &markers);
            void ShowPerceptionArrow(const FusionArrow::Ptr &arrows);
            void PushSinglePerceptionArrowMarker(const FusionArrow::Ptr &fusionArrow, const uint16_t id,
                                                 visualization_msgs::msg::MarkerArray &markers);
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fusionOdomArrowMarkerPub_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fusionPerceptionArrowMarkerPub_;
#endif

            rclcpp::Subscription<mpc_prk_msgs::msg::MsgSmHmi2Pnc>::SharedPtr parkStateMaschine2PsfSub_;
            rclcpp::Subscription<vehicle_msgs::msg::CanCopyCp2Ap50msStruct>::SharedPtr canForDoorsStatusSub_;

        private:
            // status
            bool isParking_ = false;
            bool isDoorStatusOk_ = false;

            // local map param
            const double mapPercesion_ = 0.05; // m
            const double mapRange_ = 15.0;     // m

            // local grid map
            AvpeGridMapImg::Ptr gMapPtr_;

            // id
            int arrowId_;
            Odometry preOdom_;
            std::list<FusionArrow::Ptr> trackArrows_;

            const double arrowGBLength_ = 3.0; // 国标长度
            // 宽度有四种尺度 0.45 0.75 0.9 1.35
            const double arrowGBMinWidth_ = 0.45; // 国标宽度最小值
            const double arrowGBMaxWidth_ = 1.35; // 国标宽度最大值

            std::vector<StaticElement> laneLines_;

            FixedSizeDeque<FusionArrow::Ptr> multiFrameArrows_{5};
        };
    }
}
#endif // ARROW_FUSION
