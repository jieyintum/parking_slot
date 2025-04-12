#ifndef FUSION_PARKING_STATIC_ELEMENT_NODE_H_
#define FUSION_PARKING_STATIC_ELEMENT_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#if DEBUG_MODE
#include <visualization_msgs/msg/marker_array.hpp>
#endif
#include <unordered_map>
#include <parameter_info_msgs/msg/parameter_info.hpp>
#include "mpc_prk_msgs/msg/parking_sm2_driving_sm.hpp"
#include "msg_buffer/odom_buffer.h"
#include "frame_manager.h"
#include "inner_msgs/static_elem.h"
#include "pld_fusion/pld_fusion.h"
#include "arrow_fusion/arrow_fusion.h"
#include "zebra_fusion/zebra_fusion.h"

#include "speed_bump_fusion/speed_bump_fusion.h"

// #include "parameter_service/parameter_client.hpp"
#include "lane_fusion/lane_fusion.h"

namespace Fusion
{
    namespace PSE
    {

        class MinimalSubscriber : public rclcpp::Node
        {
        public:
            MinimalSubscriber() : Node("minimal_subscriber") {}
        };

        class PseNode : public rclcpp::Node
        {
        public:
            PseNode() : Node("fusion_pse_node"), lastPldTime_(this->get_clock()->now()), lastOdomTime_(this->get_clock()->now())
            {
                Start();
            }
            ~PseNode()
            {
                Stop();
            }

        private:
            void Start();

            void Stop();

            void Register();

            void OdomCallBack(const OdometryMsgPtr msgPtr);

            void AvpeCallBack(const AvpeMsgPtr msgPtr);

            void SegCallBack(const AvpeSegArrayMsgPtr msgPtr);

            void PldCallBack(const PerceptSlotMsgPtr msgPtr);

            void USSCallBack(const uss_MsgPtr msgPtr);
            void PrkSmP0010msCallback(const mpc_prk_msgs::msg::ParkingSM2DrivingSM::SharedPtr msgPtr);

            void drDiagnosticCallback(const DiagnosticArrayMsg::SharedPtr msgPtr);
            void pldTrackingCallBack(const SlotsListMsgPtr msgPtr);
            void fsCallBack(const FSMsgPtr msgPtr);

            void PillarAndWallCallBack(const StaiticElementsMsgPtr msgPtr);
            void Process();

            void Publish();

            void PldDiagnostic();
            void PsfDiagnostic();

            void AppendStaticElemMsg(const AvpeArrowFrame &avpeArrowFrame, StaticElements &staticElems);
            void AppendStaticElemMsg(const AvpeZebraFrame &avpeZebraFrame, StaticElements &staticElems);
            void AppendStaticElemMsg(const AvpeSpeedBumpFrame &avpeSpeedBumpFrame, StaticElements &staticElems);

            void AppendStaticElemMsg(const AvpeLaneFrame &avpeLaneFrame, StaticElements &staticElems);

            void AppendWallPillars(StaticElements &staticElems);

#if DEBUG_MODE

            void ShowDetectPld(const PerceptSlot::Ptr &pldPtr);

            void ShowUSSSlot(const UssMsg::Ptr &ussPtr);

            void ShowDetectAvpe(const Avpe::Ptr &avpPtr);
            //    void ShowDetectAvpe(const Avpe::Ptr& avpPtr);

            void ShowDetectAvpe(const AvpeSegArray::Ptr &avpPtr);

            void PushSegMarker(const AvpeSegmentation &avpSeg, const uint16_t id,
                               visualization_msgs::msg::MarkerArray &markers);

            void PushSinglePldMarker(const PLD_DL_Result &slot, const uint16_t id, visualization_msgs::msg::MarkerArray &markers);

            void PushZebraSegMarker(const AvpeSegmentation &avpSeg, const uint16_t id,
                                    visualization_msgs::msg::MarkerArray &markers);

            void PushSinglePldEdgeMarker(const PLD_DL_Result &slot, const uint16_t id,
                                         visualization_msgs::msg::MarkerArray &markers);

            void PushSinglePldBlockMarker(const PLD_DL_Result &slot, const uint16_t id,
                                          visualization_msgs::msg::MarkerArray &markers);

            void ShowVisionFreespacePoint(const AvpeFreespace::Ptr &avpeMsg, visualization_msgs::msg::MarkerArray &markers);

            void ShowFusionStaticElements(const StaticElements &elements);

            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr perSlotMarkerPub_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ussPerSlotMarkerPub_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr perAvpeMarkerPub_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fusStaticElePub_;
#endif

        private:
            rclcpp::Subscription<OdometryMsg>::SharedPtr subOdom_;
            rclcpp::Subscription<PerceptSlotMsg>::SharedPtr subPerSlot_;
            rclcpp::Subscription<uss_msg>::SharedPtr subUSSInfo_;
            rclcpp::Subscription<AvpeMsg>::SharedPtr subAvpe_;
            rclcpp::Subscription<AvpeSegArrayMsg>::SharedPtr subAvpeSeg_;
            rclcpp::Subscription<mpc_prk_msgs::msg::ParkingSM2DrivingSM>::SharedPtr subSmDriving_;

            rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subDrDiagnostic_;

            rclcpp::Subscription<SlotsListMsg>::SharedPtr subPLDTracking_;

            rclcpp::Subscription<FSMsg>::SharedPtr subFS_;

            rclcpp::Subscription<StaiticElementsMsg>::SharedPtr subWallPillars_;

            rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pldDtcPub_;

            rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr psfDtcPub_;

            rclcpp::Publisher<StaiticElementsMsg>::SharedPtr pubAvpe_;
            rclcpp::TimerBase::SharedPtr processTimer_;
            rclcpp::TimerBase::SharedPtr pubTimer_;

            FrameManager frameManager_;
            std::atomic<bool> isParkingMode_;
            TimeStamp lastPldTime_;
            TimeStamp lastOdomTime_;
            diagnostic_msgs::msg::DiagnosticArray pldDiagOut;

            uint64_t frameSeq_ = 0U;

            PldFusion::Ptr pldProc_;
            ArrowFusion::Ptr arrowProc_;
            ZebraFusion::Ptr zebraProc_;

            TimeStamp lastFSTime_;
            TimeStamp lastPldTrackingTime_;
            TimeStamp lastUSSTime_;
            diagnostic_msgs::msg::DiagnosticArray psfDiagOut;
            uint64_t psfFrameSeq_ = 0U;

            TimeStamp startTime_;
            bool isStartTimeSet = false;

            SpeedBumpFusion::Ptr speedBumpProc_;

            LaneFusion::Ptr laneProc_;
            StaticElements::Ptr newestWallPillars_ = nullptr;

            std::string vehicleModel;
            Fusion::veh_params vehicleParams;

            StaiticElementsMsg sbStaticElement;
            StaiticElementsMsg otherStaticElement;
            StaiticElementsMsg sumStaticElement;

            bool slotSwitch = true;
            bool speedBumpSwitch = true;
            bool arrowSwitch = true;
            bool laneSwitch = true;
            bool ZebraSwitch = true;
            bool wallPillarSwitch = true;
        };
    }
}
#endif
