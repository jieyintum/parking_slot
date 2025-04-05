#ifndef FUSION_PSE_FUSIONPLD_H_
#define FUSION_PSE_FUSIONPLD_H_

#include <list>
#include "mpc_prk_msgs/msg/msg_sm2_psf.hpp"
#include "mpc_prk_msgs/msg/msg_sm_hmi2_pnc.hpp"
#include "fusion_base.h"
#include "inner_msgs/uss_msgs.h"
#include "pld_fusion/pld_frame.h"
#include "vehicle_msgs/msg/can_copy_cp2_ap50ms_struct.hpp"

#include "parameter_service/parameter_client.hpp"

namespace Fusion {
namespace PSE {

class PldFusion : public FusionBase<PldFrame> {

public:
    using Ptr = std::shared_ptr<PldFusion>;
    PldFusion(rclcpp::Node& nh, FrameManager& frameManager, std::string& vehicleModel, Fusion::veh_params& vehicleParams) : FusionBase<PldFrame>(nh, frameManager)
    {
        this->vehicleParams = vehicleParams;
        this->vehicleParams.calcAfterTransfer();
        ObserveRange = std::make_shared<FusionSlot>(1, this->vehicleParams);
        imageRange = std::make_shared<FusionSlot>(0, this->vehicleParams);
        vehicleRange = std::make_shared<FusionSlot>(2, this->vehicleParams);


        nh_.declare_parameter<double>("frontAxelForParallel");
        debugParameter.frontAxelForParallel = 
            nh_.get_parameter("frontAxelForParallel").as_double();

        nh_.declare_parameter<double>("rearAxelForParallel");
        debugParameter.rearAxelForParallel =  
            nh_.get_parameter("rearAxelForParallel").as_double();

        nh_.declare_parameter<double>("frontAxelForVertical");
        debugParameter.frontAxelForVertical = 
            nh_.get_parameter("frontAxelForVertical").as_double();

        nh_.declare_parameter<double>("rearAxelForVertical");
        debugParameter.rearAxelForVertical = 
            nh_.get_parameter("rearAxelForVertical").as_double();

        nh_.declare_parameter<int>("targetSlotId");
        debugParameter.targetSlotId = 
            nh_.get_parameter("targetSlotId").as_int();
        
#if DEBUG_MODE
        nh_.declare_parameter<double>("showMarkerScale");
        debugParameter.showMarkerScale = 
            nh_.get_parameter("showMarkerScale").as_double();

        nh_.declare_parameter<int>("debugSlotId");
        debugParameter.debugSlotId = 
            nh_.get_parameter("debugSlotId").as_int();

        nh_.declare_parameter<int>("debugSlotId2");
        debugParameter.debugSlotId2 = 
            nh_.get_parameter("debugSlotId2").as_int();

        nh_.declare_parameter<bool>("verticalAsRectangleSwitch");
        debugParameter.verticalAsRectangleSwitch = 
            nh_.get_parameter("verticalAsRectangleSwitch").as_bool();

        nh_.declare_parameter<double>("maxBasePointUpdateCost");
        debugParameter.maxBasePointUpdateCost = 
            nh_.get_parameter("maxBasePointUpdateCost").as_double();

        nh_.declare_parameter<double>("minBasePointUpdateCost");
        debugParameter.minBasePointUpdateCost = 
            nh_.get_parameter("minBasePointUpdateCost").as_double();

        if(vehicleModel == "ES33") {
            std::cout<<"es33 param"<<"\n";
            nh_.declare_parameter<double>("maxBufferES33");
            debugParameter.maxBuffer = 
                nh_.get_parameter("maxBufferES33").as_double();      
        }
        else {
            std::cout<<"ep35 param"<<"\n";
            nh_.declare_parameter<double>("maxBufferEP35");
            debugParameter.maxBuffer = 
                nh_.get_parameter("maxBufferEP35").as_double();
        }

        nh_.declare_parameter<double>("addBufferXThreshold");
        debugParameter.addBufferXThreshold = 
            nh_.get_parameter("addBufferXThreshold").as_double();

        nh_.declare_parameter<double>("depthCornerRatio");
        debugParameter.depthCornerRatio = 
            nh_.get_parameter("depthCornerRatio").as_double();

        nh_.declare_parameter<bool>("addBuffer");
        debugParameter.addBuffer = 
            nh_.get_parameter("addBuffer").as_bool();

        nh_.declare_parameter<double>("baseCostRatio");
        debugParameter.baseCostRatio = 
            nh_.get_parameter("baseCostRatio").as_double();

        nh_.declare_parameter<bool>("showOpenLine");
        debugParameter.showOpenLine = 
            nh_.get_parameter("showOpenLine").as_bool();

        nh_.declare_parameter<bool>("isUSSDeleteOpen");
        debugParameter.isUSSDeleteOpen = 
            nh_.get_parameter("isUSSDeleteOpen").as_bool();

        nh_.declare_parameter<bool>("isDepthsKeepWork");
        debugParameter.isDepthsKeepWork = 
            nh_.get_parameter("isDepthsKeepWork").as_bool();

        nh_.declare_parameter<double>("covErrorRatio");
        debugParameter.covErrorRatio = 
            nh_.get_parameter("covErrorRatio").as_double();

        std::cout<<"rectangle switch: "<<debugParameter.verticalAsRectangleSwitch<<"\n";

        fusionSlotMarkerPub_ = nh_.create_publisher<visualization_msgs::msg::MarkerArray>("/pld_trackingOutputSlot", 1);
        ussSlotsPub_ = nh_.create_publisher<SlotsListMsg>("/cem/fusion/uss_slots", 1);
        slotsTheMomentIntoParkInMode_pub = nh_.create_publisher<visualization_msgs::msg::MarkerArray>("/viz/pld/pldSlotsTheMomentIntoParkInMode", 1);
#endif
        parkStateMaschine2PsfSub_ = nh_.create_subscription<mpc_prk_msgs::msg::MsgSmHmi2Pnc>("mpc_prk_sm2pnc", 10, std::bind(&PldFusion::ParkSmCallback, this, std::placeholders::_1));
        canForDoorsStatusSub_ = nh_.create_subscription<vehicle_msgs::msg::CanCopyCp2Ap50msStruct>("/mpc_cp2ap_50ms_CanCopyCp2Ap50msStruct", 10, std::bind(&PldFusion::CANCallBack, this, std::placeholders::_1)); 

        slotsPub_ = nh_.create_publisher<SlotsListMsg>("/cem/tracking/pld_tracking_slots", 1);
        MergedSlotsPub_ = nh_.create_publisher<SlotsListMsg>("/cem/fusion/parking_slot", 1);
    }
    ~PldFusion() = default;

    void Process(PldFrame& frame) override
    {
        std::unique_lock<std::mutex> lock(mtx);
        if (MakeFrame(frame)) {
            Detect(frame);
            Track(frame);
            Output(frame);
#if DEBUG_MODE
            Debug(frame);
#endif
        }

        if(frame.isParkSystemWork == false) {
            std::cout<<"pld node reset"<<"\n";
            resetPldNode();
        }
    }

private:
    bool MakeFrame(PldFrame& frame) override;
    void Detect(PldFrame& frame) override;
    void Track(PldFrame& frame) override;
    void Output(PldFrame& frame) override;
    void Debug(PldFrame& frame) override;
    void Publish(PldFrame& frame) override;

    void ParkSmCallback(const mpc_prk_msgs::msg::MsgSmHmi2Pnc::SharedPtr msg);
    void CANCallBack(const vehicle_msgs::msg::CanCopyCp2Ap50msStruct::SharedPtr msg);
    void CalculateOtherCorners(FusionSlot::Ptr& tSlot);

    void ussSlotInputProcess(CUssSlotsList& ussSlots_, PldFrame& frame, const bool isInParkIn);
    std::vector<FusionSlot::Ptr> GenerateFusionSlot(const PldFrame& frame);
    void AlignSlotWithFreespaceType(const FusionSlot::Ptr& slot, const VisionFreespace::Ptr& vsFs);
    void AlignSlotWithAvpe(const FusionSlot::Ptr& slot, const visionObjVector::Ptr& vsObject);
    void EstimateOccupiedType(const FusionSlot::Ptr& trackSlot);
    void EstimateOpenLinePosition(const FusionSlot::Ptr& trackSlot);
    void CalculateSlotAngle(FusionSlot::Ptr& fusionSlot);

    void CalculatePerceptErrorToCenter(const FusionSlot::Ptr& perSlot, const FusionSlot::Ptr& trackSlot);

    void FusionMatchSlots(PldFrame& frame, std::list<FusionSlot::Ptr>& trackSlots);
    double CalculateWeightOfPerceptSlots(const FusionSlot::Ptr& tSlot);
    void CalculateCovarianceOfTrackSlots(FusionSlot::Ptr& trackSlot);
    void EstimatePldStatus(FusionSlot::Ptr& trackSlot);
    void FusionBlockPoint(PldFrame& frame, FusionSlot::Ptr& trackSlot);
    bool isBlockInGoodObservationBlock(const FusionSlot::Ptr& fusionSlot, PldFrame& frame);

    void GenerateNewTrackSlots(const std::vector<FusionSlot::Ptr>& noMatchSlots, CSlotsList& trackSlots);

    void FusionSlotEdges(const FusionSlot::Ptr detectSlot, const double fusionScale, FusionSlot::Ptr& trackSlot);
    void FixOutput(std::list<FusionSlot::Ptr>& slots, const Odometry& odom);
    void AlignAjoinedSlots(std::list<FusionSlot::Ptr>& slots);
    void CalculateBlockDepth(FusionSlot::Ptr& fusionSlot);
    void AdjustSlotBlock(FusionSlot::Ptr& fusionSlot);
    void GetSlotCoordTransform(const FusionSlot::Ptr& fusionSlot, Eigen::Quaterniond& quat_sb, Eigen::Vector3d& p_sb);
    void FindAjoinRightSlot(const FusionSlot::Ptr& slot, std::vector<FusionSlot::Ptr>& ajoinSlots);
    void MakeAlign(const std::vector<FusionSlot::Ptr>& ajoinSlots);
    void AdaptOutputForPnc(FusionSlot::Ptr& slot);

    void slotTypeHandle(const FusionSlot::Ptr tSlot);

    SlotsListMsg PubSlot(
        const std::vector<FusionSlot::Ptr>& slots,
        const Odometry& poseOb);

    void slotMergeProcess(
        const std::list<FusionSlot::Ptr> pldSlots,
        const std::list<FusionSlot::Ptr> ussSlots,
        CSlotsList& mergedSlots,
        const Odometry poseOb);
    void fusionProcess(
        const std::list<FusionSlot::Ptr> pldSlots,
        const std::list<FusionSlot::Ptr> ussSlots,
        CSlotsList& mergedSlots);
    bool isMergeSlotExit(
        const FusionSlot::Ptr mergedSlots,
        const std::list<FusionSlot::Ptr> pldSlots,
        const std::list<FusionSlot::Ptr> ussSlots);

    void oldMatchedInfoProcess(
        const std::list<FusionSlot::Ptr> pldSlots,
        const std::list<FusionSlot::Ptr> ussSlots);
    void oldMatchedInfoProcessForPld(
        const FusionSlot::Ptr slotA,
        const std::list<FusionSlot::Ptr> slotsListB);
    void findPartnerSlots(
        const std::list<FusionSlot::Ptr> pldSlots,
        const std::list<FusionSlot::Ptr> ussSlots);
    void findPartnerSlotForPld(
        const FusionSlot::Ptr pldSlot,
        const std::list<FusionSlot::Ptr> ussSlots);
    void fusionInfoReset(
        const std::list<FusionSlot::Ptr> pldSlots,
        const std::list<FusionSlot::Ptr> ussSlots);
    bool isConflictWithOtherSlot(
        const std::list<FusionSlot::Ptr> mergedSlotsList,
        const FusionSlot::Ptr slot);

    void addOutputSlot(
        const FusionSlot::Ptr& slot,
        const std::list<FusionSlot::Ptr> oldOutputSlots,
        std::list<FusionSlot::Ptr>& newOutputSlots);

    void resetPldNode();

    bool IsConflictWithFSAndPld(const FusionSlot::Ptr slot, const FSStruct::Ptr fsStruct, const std::list<FusionSlot::Ptr> pldSlots);
    bool slotAdjustUsingFs(const FusionSlot::Ptr slot, const FSStruct::Ptr fsStruct);
    bool processWithFSPoint(const float minDepth, const float tau0Middle, const float tau1Middle, 
                      float& tauOut0, float& tauOut1, const FusionSlot::Ptr slot, const Eigen::Vector3d ObjworldPoint);

    void ParallelDepthHandle(const FusionSlot::Ptr slot);
    void VertcialInclineDepthHandle(const FusionSlot::Ptr slot);

    bool isInsideObserveRange(const FusionSlot::Ptr ObserveRange, const FusionSlot::Ptr slot);


    void matchAlgo(PldFrame& frame, std::vector<FusionSlot::Ptr>& noMatchSlots);
    bool isAllInfoofMatchedPSBeenProcessedFinished(const std::vector<FusionSlot::Ptr> detectSlots);
    bool isConflictWithOtherTrackSlot(const std::list<FusionSlot::Ptr> trackSlotsList, const FusionSlot::Ptr slot);
    bool getMatchedInfo(const FusionSlot::Ptr trackSlot,const FusionSlot::Ptr newSlot);
    bool checkIfOverlapOK(
        const Fusion::CTrackLine::Ptr line1, 
        const Fusion::CTrackLine::Ptr line2, 
        float& f_minOverlapRatio_f);
    bool isLinesMatched(
        const Fusion::CTrackLine::Ptr line1,
        const Fusion::CTrackLine::Ptr line2,
        float& f_errorOfOrthDist_f,
        float& f_minOverlapRatio_f);
    void p0p1ErrorHandle(
        const FusionSlot::Ptr tSlot,
        const std::list<FusionSlot::Ptr> mergedSlotsList,
        const int targetID);


    void openLineCalc(
        std::list<FusionSlot::Ptr>& trackSlots,
        PldFrame& frame);
    void calcWithEgoWheelStop(const FusionSlot::Ptr targetSlot);
    void calcWithFSStruct(
        const FusionSlot::Ptr slot,
        const FSStruct::Ptr fsStruct);
    void calcWithFSPoint(const FusionSlot::Ptr slot, const Eigen::Vector3d fsPoint);
    void calcWithOtherSlot(
        const FusionSlot::Ptr targetSlot,
        const std::list<FusionSlot::Ptr> trackSlots);
    void neighborLinesDispatch(
        const FusionSlot::Ptr slotA, 
        const FusionSlot::Ptr slotB);

    void neighborLineHandle(
        const Fusion::CTrackLine::Ptr lineA,
        const Fusion::CTrackLine::Ptr oppositeLineA,
        const Fusion::CTrackLine::Ptr lineB,
        const Fusion::CTrackLine::Ptr oppositeLineB);

    void finalOpenLineSelect(const FusionSlot::Ptr targetSlot);
    void finalBattle(
        std::array<Fusion::CTrackLine::Ptr, 2>& lines,
        std::array<float, 2> costs,
        const FusionSlot::Ptr targetSlot);
    float calcLineCostAsOpenLine(
        const Eigen::Vector3d startPoint,
        const Eigen::Vector3d endPoint);
    int pairLinesHandle(
        const Fusion::CTrackLine::Ptr lowerCostLine, 
        const Fusion::CTrackLine::Ptr higherCostLine);

    void isCornerAsTrunctionPoint(
        const FusionSlot::Ptr imageRange,
        FusionSlot::SlotCorner& corner);

    void truePointHandle(const FusionSlot::Ptr& fusionSlot);

    void checkCornerConfidenceMatchedSlot(const FusionSlot::Ptr& fusionSlot);

#if DEBUG_MODE
    SlotsListMsg PubUssSlot(
        const std::list<FusionSlot::Ptr>& ussSlots, 
        const Odometry& poseOb, 
        const int mode);
    void ShowFusionPld(const std::list<FusionSlot::Ptr>& slots);
    void ShowSlotsIntoParkinMode(const std::list<FusionSlot::Ptr>& slots);
    void PushSingleFusionPldMarker(const FusionSlot::Ptr& fusionSlot, visualization_msgs::msg::MarkerArray& markers, const int mode);
    void SetAvailableMarker(const FusionSlot::Ptr& fusionSlot, visualization_msgs::msg::Marker& marker);
    void SetOccupiedMarker(const FusionSlot::Ptr& fusionSlot, visualization_msgs::msg::Marker& marker);
    void PushSlotId(const FusionSlot::Ptr& fusionSlot, visualization_msgs::msg::MarkerArray& markers);
    void PushSlotBlock(const FusionSlot::Ptr& fusionSlot, visualization_msgs::msg::MarkerArray& markers, const int mode);
    void PushEdges(const FusionSlot::Ptr& fusionSlot, const uint16_t id, visualization_msgs::msg::MarkerArray& markers);
    void PushSlotDirection(const FusionSlot::Ptr& fusionSlot, visualization_msgs::msg::MarkerArray& markers);
    void calcPointsDiffInSlotCoordinate(
        const FusionSlot::Ptr outSlot, 
        const FusionSlot::Ptr tSlot,
        const float tauP0, 
        const float tauP1,
        const Eigen::Vector3d worldP0,
        const Eigen::Vector3d worldP1);
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fusionSlotMarkerPub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr slotsTheMomentIntoParkInMode_pub;
    rclcpp::Publisher<SlotsListMsg>::SharedPtr ussSlotsPub_;
#endif



private:
    CSlotsList trackSlots_;

    CUssSlotsList ussSlots_;

    CSlotsList mergedSlots_;
#if DEBUG_MODE
    CSlotsList slotsIntoParkInMode_;
#endif
    std::list<FusionSlot::Ptr> outputSlots_;

    FusionSlot::Ptr ObserveRange;


    FusionSlot::Ptr imageRange;

    FusionSlot::Ptr vehicleRange;

    bool isUpdate_ = false;
    bool isMoving_ = false;
    Odometry lastOdom_;

    rclcpp::Subscription<mpc_prk_msgs::msg::MsgSmHmi2Pnc>::SharedPtr parkStateMaschine2PsfSub_;

    rclcpp::Subscription<vehicle_msgs::msg::CanCopyCp2Ap50msStruct>::SharedPtr canForDoorsStatusSub_;

    std::atomic<bool> isParking_;
    bool isDoorStatusOk_ = false;
    bool isLastFrameParkingIn_ = false;
    rclcpp::Publisher<SlotsListMsg>::SharedPtr slotsPub_;
    rclcpp::Publisher<SlotsListMsg>::SharedPtr MergedSlotsPub_;

    Fusion::PSE::debugParams debugParameter;

    std::string vehicleModel;
    Fusion::veh_params vehicleParams;
};
}
}
#endif