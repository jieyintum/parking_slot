#include "pse_node.h"

int g_circle_i(0);

using namespace std::chrono_literals;
namespace {
    constexpr uint8_t MSG_BUFFER_NUM = 5U;
}
namespace Fusion {
namespace PSE {

void PseNode::Start()
{
    Register();

    pldProc_ = std::make_shared<PldFusion>(*this, frameManager_, vehicleModel, vehicleParams);
    arrowProc_ = std::make_shared<ArrowFusion>(*this, frameManager_);
    zebraProc_ = std::make_shared<ZebraFusion>(*this, frameManager_);
    speedBumpProc_ = std::make_shared<SpeedBumpFusion>(*this, frameManager_);
    laneProc_ = std::make_shared<LaneFusion>(* this, frameManager_);
}

void PseNode::Stop()
{

}

void PseNode::Register()
{
    auto client_node = std::make_shared<Fusion::PSE::MinimalSubscriber>();
    auto client = std::make_shared<ParameterService::Client>(client_node, "/vehicle_parameters"); //service name要与server端保持一致
    client->GetParameter(); //调用此接口时，线程会阻塞等待，直到取到参数

    client->ParseParameter("vehicleLength", vehicleParams.g_vehicleLength);
    client->ParseParameter("vehicleWidth", vehicleParams.g_vehicleWidth);
    client->ParseParameter("rearCenter2RearCar", vehicleParams.g_rearAxleToVehicleRear);
    client->ParseParameter("rearAxleCenter", vehicleParams.g_vehicleOrignal);
    client->ParseParameter("vehicleModel", vehicleModel);

    std::cout<<"vehicleParams.g_vehicleLength: "<<vehicleParams.g_vehicleLength<<"\n";
    std::cout<<"vehicleParams.g_vehicleWidth: "<<vehicleParams.g_vehicleWidth<<"\n";
    std::cout<<"vehicleParams.g_rearAxleToVehicleRear: "<<vehicleParams.g_rearAxleToVehicleRear<<"\n";
    std::cout<<"vehicleParams.g_vehicleOrignalXInImage: "<<(int)vehicleParams.g_vehicleOrignal[0]<<"\n";
    std::cout<<"vehicleParams.g_vehicleOrignalYInImage: "<<(int)vehicleParams.g_vehicleOrignal[1]<<"\n";
    std::cout<<"vehicleModel: "<<vehicleModel<<"\n";


    this->declare_parameter<bool>("slotSwitch");
    slotSwitch = this->get_parameter("slotSwitch").as_bool();
    std::cout<<"slotSwitch: "<<slotSwitch<<"\n";

    this->declare_parameter<bool>("speedBumpSwitch");
    speedBumpSwitch = this->get_parameter("speedBumpSwitch").as_bool();
    std::cout<<"speedBumpSwitch: "<<speedBumpSwitch<<"\n";

    this->declare_parameter<bool>("arrowSwitch");
    arrowSwitch = this->get_parameter("arrowSwitch").as_bool();
    std::cout<<"arrowSwitch: "<<arrowSwitch<<"\n";

    this->declare_parameter<bool>("laneSwitch");
    laneSwitch = this->get_parameter("laneSwitch").as_bool();
    std::cout<<"laneSwitch: "<<laneSwitch<<"\n";

    this->declare_parameter<bool>("ZebraSwitch");
    ZebraSwitch = this->get_parameter("ZebraSwitch").as_bool();
    std::cout<<"ZebraSwitch: "<<ZebraSwitch<<"\n";

    this->declare_parameter<bool>("wallPillarSwitch");
    wallPillarSwitch = this->get_parameter("wallPillarSwitch").as_bool();
    std::cout<<"wallPillarSwitch: "<<wallPillarSwitch<<"\n";

    subOdom_ = this->create_subscription<OdometryMsg>("/cem/loc/odom_dr", MSG_BUFFER_NUM, std::bind(&PseNode::OdomCallBack, this, std::placeholders::_1));
    subPerSlot_ = this->create_subscription<PerceptSlotMsg>("/cem/vision/pld/detect_objects", MSG_BUFFER_NUM, std::bind(&PseNode::PldCallBack, this, std::placeholders::_1));
    subUSSInfo_ = this->create_subscription<uss_msg>("/sensor/uss/uss", MSG_BUFFER_NUM, std::bind(&PseNode::USSCallBack, this, std::placeholders::_1));
    subAvpe_ = this->create_subscription<AvpeMsg>("/cem/vision/avpe/postpro_segs", MSG_BUFFER_NUM, std::bind(&PseNode::AvpeCallBack, this, std::placeholders::_1));
    subSmDriving_ = this->create_subscription<mpc_prk_msgs::msg::ParkingSM2DrivingSM>("/mpc_interface_prk_sm_p0010ms_driving",
        MSG_BUFFER_NUM, std::bind(&PseNode::PrkSmP0010msCallback, this, std::placeholders::_1));

    subDrDiagnostic_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/cem/dtc/odom_dr",
        MSG_BUFFER_NUM, std::bind(&PseNode::drDiagnosticCallback, this, std::placeholders::_1));
    
    subPLDTracking_ = this->create_subscription<SlotsListMsg>("/cem/tracking/pld_tracking_slots",
        MSG_BUFFER_NUM, std::bind(&PseNode::pldTrackingCallBack, this, std::placeholders::_1));

    subFS_ = this->create_subscription<FSMsg>("/cem/fusion/freespace_points", 
        MSG_BUFFER_NUM, std::bind(&PseNode::fsCallBack, this, std::placeholders::_1));

    pubAvpe_ = this->create_publisher<StaiticElementsStruct>("/cem/fusion/static_elements_avpe", 1);

    processTimer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration(200ms), std::bind(&PseNode::Process, this));
    pubTimer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration(40ms), std::bind(&PseNode::Publish, this));

    subAvpeSeg_ = this->create_subscription<AvpeSegArrayMsg>("/cem/vision/avpe/postpro_segment_map", MSG_BUFFER_NUM,
                    std::bind(& PseNode::SegCallBack, this, std::placeholders::_1));

    subWallPillars_ = this->create_subscription<StaiticElementsMsg>("/cem/fusion/static_elements", MSG_BUFFER_NUM, std::bind(& PseNode::PillarAndWallCallBack, this, std::placeholders::_1));
    

    pldDtcPub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/cem/dtc/fusion_pse", MSG_BUFFER_NUM);
    psfDtcPub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/cem/dtc/fusion_psf", MSG_BUFFER_NUM);
#if DEBUG_MODE
    perSlotMarkerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pld_slotVehicleCoord", 1);
    ussPerSlotMarkerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/uss_perception_slot", 1);
    perAvpeMarkerPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/avpe_points", 1);
    fusStaticElePub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/static_elements", 1);
#endif
}

void PseNode::SegCallBack(const AvpeSegArrayMsgPtr msgPtr)
{
    auto segPtr = std::make_shared<AvpeSegArray>(msgPtr);
    frameManager_.Push(segPtr);

#if DEBUG_MODE
//    ShowDetectAvpe(segPtr);
#endif
}

void PseNode::fsCallBack(const FSMsgPtr msgPtr)
{
    const auto fsStructPtr = std::make_shared<FSStruct>(msgPtr);
    frameManager_.Push(fsStructPtr);
    lastFSTime_ = this->get_clock()->now();
}


void PseNode::pldTrackingCallBack(const SlotsListMsgPtr msgPtr)
{
    const auto trackingSlotsPtr = std::make_shared<FusionSlots>(msgPtr);
    //frameManager_.Push(trackingSlotsPtr);
    lastPldTrackingTime_ =  this->get_clock()->now(); //trackingSlotsPtr->timestamp;
}

void PseNode::AvpeCallBack(const AvpeMsgPtr msgPtr)
{
    auto avpePtr = std::make_shared<Avpe>(msgPtr);
    frameManager_.Push(avpePtr);

#if DEBUG_MODE
//    ShowDetectAvpe(avpePtr);
#endif
}

void PseNode::OdomCallBack(const OdometryMsgPtr msgPtr)
{
    auto odomPtr = std::make_shared<Odometry>(msgPtr);
    OdomBuffer::GetInstance().Push(odomPtr);
    lastOdomTime_ = odomPtr->timestamp;
}

void PseNode::PldCallBack(const PerceptSlotMsgPtr msgPtr)
{
    auto pldPtr = std::make_shared<PerceptSlot>(msgPtr, vehicleParams);
    frameManager_.Push(pldPtr);
    lastPldTime_ = pldPtr->timestamp;
#if DEBUG_MODE
    ShowDetectPld(pldPtr);
#endif
}

void PseNode::USSCallBack(const uss_MsgPtr msgPtr)
{
    auto ussPtr = std::make_shared<UssMsg>(msgPtr);
    frameManager_.Push(ussPtr);
    lastUSSTime_ =  this->get_clock()->now();// ussPtr->timestamp;
#if DEBUG_MODE
    ShowUSSSlot(ussPtr);
#endif
}

void PseNode::PillarAndWallCallBack(const StaiticElementsMsgPtr msgPtr)
{
    const auto pillarWallPtr = std::make_shared<StaticElements>(msgPtr, true);
    frameManager_.Push(pillarWallPtr);
}

void PseNode::PrkSmP0010msCallback(const mpc_prk_msgs::msg::ParkingSM2DrivingSM::SharedPtr msgPtr)
{
    isParkingMode_ = (msgPtr->sm_apa_indx_apaactsts == 1U);
}


void PseNode::drDiagnosticCallback(const DiagnosticArrayMsg::SharedPtr msgPtr)
{
    const auto drDiagnosticPtr = std::make_shared<Diagnostic>(msgPtr);
    frameManager_.Push(drDiagnosticPtr);
}

void PseNode::AppendStaticElemMsg(const AvpeArrowFrame& avpeArrowFrame, StaticElements& staticElems)
{
    for (size_t i = 0 ; i < avpeArrowFrame.outputArrows_.size() ; ++i) {
        staticElems.Append(avpeArrowFrame.outputArrows_[i]->ToMsg());
    }
}

void PseNode::AppendStaticElemMsg(const AvpeZebraFrame& avpeZebraFrame, StaticElements& staticElems)
{
    for (size_t i = 0 ; i < avpeZebraFrame.outputZebras_.size() ; ++i) {
        staticElems.Append(avpeZebraFrame.outputZebras_[i]->ToMsg());
    }
}


void PseNode::AppendStaticElemMsg(const AvpeSpeedBumpFrame& avpeSpeedBumpFrame,  StaticElements& staticElems) 
{
    for (size_t i = 0; i < avpeSpeedBumpFrame.output_speed_bumps_.size(); ++i) {
        staticElems.Append(avpeSpeedBumpFrame.output_speed_bumps_[i]->ToMsg());
    }
}

void PseNode::AppendStaticElemMsg(const AvpeLaneFrame& avpeLaneFrame, StaticElements& staticElems)
{
    for (size_t i = 0 ; i < avpeLaneFrame.outputLanes_.size() ; ++i) {
        staticElems.Append(avpeLaneFrame.outputLanes_[i]->ToMsg());
    }
}

void PseNode::AppendWallPillars(StaticElements& staticElems)
{
    auto newestWallPillarsTmp = frameManager_.GetNewestWallPillars();
    if (newestWallPillarsTmp != nullptr) {
        newestWallPillars_ = newestWallPillarsTmp;
    }
    if (newestWallPillars_ != nullptr) {
        staticElems.Append(* newestWallPillars_);
    }
}

void PseNode::Process()
{
    std::cout<<g_circle_i<<"\n";
    g_circle_i = (++g_circle_i == 15)?0:(g_circle_i);
    

    if(slotSwitch == true) {

    PldFrame pldFrame;
    pldFrame.isParkSystemWork = isParkingMode_;
    pldProc_->Process(pldFrame);

    }

    sumStaticElement.elements_list.clear();
    sumStaticElement.elements_number = 0;

    Avpe::Ptr sbAvpePtr = frameManager_.GetLastAvpePtr();
    if(sbAvpePtr != nullptr) {
        sbStaticElement.elements_list.clear();
        StaticElements staticElems (sbAvpePtr->GetTime());


        // speed bump.
        if(speedBumpSwitch == true) {
    
        AvpeSpeedBumpFrame avpeSpeedBumpFrame;
        speedBumpProc_->SetParkingMode(isParkingMode_);
        speedBumpProc_->Process(avpeSpeedBumpFrame);
        AppendStaticElemMsg(avpeSpeedBumpFrame, staticElems);
        sbStaticElement = staticElems.ToMsg();

        }

        sumStaticElement.elements_number += sbStaticElement.elements_number;
        sumStaticElement.header.stamp = sbStaticElement.header.stamp;
        sumStaticElement.cem_header.time_stamp = sbStaticElement.cem_header.time_stamp;

        // lane line.
        // publish
        //pubAvpe_ ->publish(staticElems.ToMsg());
    }

    const auto avpePtr = frameManager_.GetLastSegPtr();
    if (avpePtr != nullptr) {
        otherStaticElement.elements_list.clear();
        
        StaticElements staticElems(avpePtr->GetTime());

        if(ZebraSwitch == true) {

        // cross node zebra line.
        AvpeZebraFrame avpeZebraFrame;
        zebraProc_->Process(avpeZebraFrame);
        AppendStaticElemMsg(avpeZebraFrame, staticElems);

        }

        if(laneSwitch == true) {

            // lane line.
            AvpeLaneFrame avpeLaneFrame;
            laneProc_->Process(avpeLaneFrame);
            AppendStaticElemMsg(avpeLaneFrame, staticElems);
        }

        if(arrowSwitch == true) {
            
            // lane arrow.
            AvpeArrowFrame avpeArrowFrame;
            arrowProc_->SetCurFrameLanLine(staticElems.elems);
            arrowProc_->Process(avpeArrowFrame);
            AppendStaticElemMsg(avpeArrowFrame, staticElems);

        }

        if(wallPillarSwitch == true) {

        AppendWallPillars(staticElems);

        }
#if DEBUG_MODE
        ShowFusionStaticElements(staticElems);

        otherStaticElement = staticElems.ToMsg();
        
        sumStaticElement.elements_number += otherStaticElement.elements_number;
        sumStaticElement.header.stamp = otherStaticElement.header.stamp;
        sumStaticElement.cem_header.time_stamp = otherStaticElement.cem_header.time_stamp;
#endif
        //pubAvpe_->publish(staticElems.ToMsg());
    }

    for(int i = 0; i < sbStaticElement.elements_list.size(); i++) {
        sumStaticElement.elements_list.push_back(sbStaticElement.elements_list[i]);
    }

    for(int i = 0; i < otherStaticElement.elements_list.size(); i++) {
        sumStaticElement.elements_list.push_back(otherStaticElement.elements_list[i]);
    }

    pubAvpe_->publish(sumStaticElement);
}

void PseNode::Publish()
{
    pldProc_->PubOut();

    PldDiagnostic();
    PsfDiagnostic();
}

void PseNode::PsfDiagnostic()
{
    if(isParkingMode_) {
        if(isStartTimeSet == false) {
            startTime_ = this->get_clock()->now();
            isStartTimeSet = true;
        }
    }
    else {
        isStartTimeSet = false;
    }
    
    psfDiagOut.status.clear();
    psfDiagOut.header.stamp = this->get_clock()->now();
    psfDiagOut.header.frame_id = std::to_string(psfFrameSeq_);
    ++psfFrameSeq_;
    if(psfFrameSeq_ == 16) {
        psfFrameSeq_ = 0U;
    }

    if(isParkingMode_) {
        //Diagnostic drDiag;

        Diagnostic::Ptr drDiagnostic = nullptr;
        if(frameManager_.drDiagnostic_.GetNewest(drDiagnostic)) {

        }
        else{
            drDiagnostic = std::make_shared<Diagnostic>();
        }
        Diagnostic::Ptr pldDiagnostic = std::make_shared<Diagnostic>(pldDiagOut);

        Diagnostic::Ptr ussDiagnostic = std::make_shared<Diagnostic>();




        TimeStamp currTime(psfDiagOut.header.stamp);
        
        if(currTime.Sec() - startTime_.Sec() < 10.0) {
            psfDtcPub_->publish(psfDiagOut);
            return;
        }
        
        diagnostic_msgs::msg::KeyValue keyValue;
        keyValue.key = "source";

        if(currTime.Sec() - lastPldTrackingTime_.Sec() > 3.0) {
            keyValue.value = "03090001";
            pldDiagnostic->addKeyValue(keyValue);
        }

        if (currTime.Sec() - lastOdomTime_.Sec() > 3.0) {
            keyValue.value = "03320004";
            drDiagnostic->addKeyValue(keyValue);
        }
        if (currTime.Sec() - lastUSSTime_.Sec() > 3.0) {
            keyValue.value = "01070001";
            ussDiagnostic->addKeyValue(keyValue);
        }

        Diagnostic sumDiag;
        Diagnostic::Ptr sumDiagnostic = std::make_shared<Diagnostic>(sumDiag);
        for(int i = 0; i < pldDiagnostic->keyValuesList_.size(); i++) {
            sumDiagnostic->addKeyValue(pldDiagnostic->keyValuesList_[i]);
        }
        if(drDiagnostic != nullptr) {
            for(int i = 0; i < drDiagnostic->keyValuesList_.size(); i++) {
                sumDiagnostic->addKeyValue(drDiagnostic->keyValuesList_[i]);
            }
        }
        if(ussDiagnostic != nullptr) {
            for(int i = 0; i < ussDiagnostic->keyValuesList_.size(); i++) {
                sumDiagnostic->addKeyValue(ussDiagnostic->keyValuesList_[i]);
            }
        }
        
        diagnostic_msgs::msg::DiagnosticStatus l_status_e;
        l_status_e.name = "fusion_psf";
        for(int i = 0; i < sumDiagnostic->keyValuesList_.size(); i++) {
            l_status_e.values.push_back(sumDiagnostic->keyValuesList_[i]);
        }
        

        if( (drDiagnostic->keyValuesList_.size() != 0)
         || ((pldDiagnostic->keyValuesList_.size() != 0) 
          && (ussDiagnostic->keyValuesList_.size() != 0)) ) {
            l_status_e.level = 0x00000111;
            l_status_e.message = "pld fusion unavailable all";
            l_status_e.hardware_id = "03080001";
            psfDiagOut.status.push_back(l_status_e);
        }
        else if(pldDiagnostic->keyValuesList_.size() != 0) {
            l_status_e.level = 0x00000001;
            l_status_e.message = "pld fusion vision slots unavailable";
            l_status_e.hardware_id = "03080002";
            psfDiagOut.status.push_back(l_status_e);
        }
        else if(ussDiagnostic->keyValuesList_.size() != 0) {
            l_status_e.level = 0x00000001;
            l_status_e.message = "pld fusion uss slots unavailable";
            l_status_e.hardware_id = "03080003";
            psfDiagOut.status.push_back(l_status_e);
        }
        else {

        }

    }

    psfDtcPub_->publish(psfDiagOut);

}



void PseNode::PldDiagnostic()
{
    //diagnostic_msgs::msg::DiagnosticArray diagOut;
    pldDiagOut.status.clear();
    pldDiagOut.header.stamp = this->get_clock()->now();
    pldDiagOut.header.frame_id = std::to_string(frameSeq_);

    TimeStamp currTime(pldDiagOut.header.stamp);
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fusion_pse";
    if (isParkingMode_) {
        if (currTime.Sec() - lastPldTime_.Sec() > 3.0) {
            status.level = 0b00000111;
            status.message = "pld lost";
            status.hardware_id = "03090001";
            diagnostic_msgs::msg::KeyValue value;
            value.key = "source";
            value.value = "02010011";
            status.values.push_back(value);
            pldDiagOut.status.push_back(status);
        }
        if (currTime.Sec() - lastOdomTime_.Sec() > 0.5) {
            status.level = 0b00000111;
            status.message = "odom lost";
            status.hardware_id = "03090001";
            diagnostic_msgs::msg::KeyValue value;
            value.key = "source";
            value.value = "03320004";
            status.values.push_back(value);
            pldDiagOut.status.push_back(status);
        }
        if (!OdomBuffer::GetInstance().Empty()) {
            const auto& lastOdom = OdomBuffer::GetInstance().Back();
            if (!((lastOdom->status == 2) && (lastOdom->source == 3 || lastOdom->source == 2))) {
                status.level = 0b00000111;
                status.message = "odom is error";
                status.hardware_id = "03090001";
                diagnostic_msgs::msg::KeyValue value;
                value.key = "source";
                value.value = "03320003";
                status.values.push_back(value);
                pldDiagOut.status.push_back(status);
            }
        }
    } else {
        lastPldTime_ = currTime;
    }
    if (pldDiagOut.status.empty()) {
        status.level = 0;
        status.message = "fusion pse is good";
        status.hardware_id = "03090000";
        pldDiagOut.status.push_back(status);
    }

    ++frameSeq_;

    pldDtcPub_->publish(pldDiagOut);
}

#if DEBUG_MODE

void PseNode::ShowDetectPld(const PerceptSlot::Ptr& pldPtr)
{
    visualization_msgs::msg::MarkerArray markers;

    for (uint16_t i = 0U ; i < pldPtr->frame.slot_num ; ++i) {
        const auto& slot = pldPtr->frame.DL_Result_one_frame[i];
        PushSinglePldMarker(slot, i, markers);
        PushSinglePldEdgeMarker(slot, i, markers);
        PushSinglePldBlockMarker(slot, i, markers);
    }

    perSlotMarkerPub_->publish(markers);
}

void PseNode::ShowUSSSlot(const UssMsg::Ptr& ussPtr)
{
    visualization_msgs::msg::MarkerArray markers = ussPtr->ToMarkerArray(this->get_clock()->now());
    ussPerSlotMarkerPub_->publish(markers);
}

void PseNode::ShowDetectAvpe(const Avpe::Ptr& avpePtr)
{
    visualization_msgs::msg::MarkerArray markers;

    ShowVisionFreespacePoint(avpePtr->freespace, markers);
    perAvpeMarkerPub_->publish(markers);
}

void PseNode::ShowDetectAvpe(const AvpeSegArray::Ptr& segPtr)
{
    visualization_msgs::msg::MarkerArray markers;
    uint16_t id = 0;
    for (const auto& seg : segPtr->segmentations) {
        PushSegMarker(seg.second, id, markers);
        ++id;
    }
    perAvpeMarkerPub_->publish(markers);
}

void PseNode::PushZebraSegMarker(const AvpeSegmentation& avpSeg, const uint16_t id,
                                 visualization_msgs::msg::MarkerArray& markers)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "zebra_seg";
    marker.id = id;
    marker.type = 8;
    marker.action = 0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    std_msgs::msg::ColorRGBA color;
    color.r = 0;
    color.g = 1;
    color.b = 0;
    color.a = 1;
    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0);


    for (uint32_t i = 1U ; i < avpSeg.region.size() ; ++i) {
        geometry_msgs::msg::Point point;
        point.x = avpSeg.region[i].point.x();
        point.y = avpSeg.region[i].point.y();
        point.z = 0.0;
        marker.points.push_back(std::move(point));
        marker.colors.push_back(std::move(color));
    }
    markers.markers.emplace_back(marker);
}


void PseNode::PushSinglePldMarker(const PLD_DL_Result& slot, const uint16_t id,
                                  visualization_msgs::msg::MarkerArray& markers)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "percept";
    marker.id = id;
    marker.type = 4;
    marker.action = 0;

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    std_msgs::msg::ColorRGBA color;
    color.r = 1;
    color.g = 1;
    color.b = 0;
    color.a = 1;
    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);


    for (uint32_t i = 1U ; i < 5 ; ++i) {
        geometry_msgs::msg::Point point;
        const auto index = i % 4U;
        point.x = slot.slot_GroundPts[index].CornerPot.x;
        point.y = slot.slot_GroundPts[index].CornerPot.y;
        point.z = 0.0;
        marker.points.push_back(std::move(point));
        marker.colors.push_back(std::move(color));
    }

    markers.markers.emplace_back(marker);
}

void PseNode::PushSinglePldBlockMarker(const PLD_DL_Result& slot, const uint16_t id,
                                       visualization_msgs::msg::MarkerArray& markers)
{
    if (slot.block_GroundPt.Block_type == 0) {
        return;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "block";
    marker.id = id;
    marker.type = 8;
    marker.action = 0;

    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;

    std_msgs::msg::ColorRGBA color;
    color.r = 1;
    color.g = 1;
    color.b = 0;
    color.a = 1;
    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    geometry_msgs::msg::Point point;
    point.x = slot.block_GroundPt.BlockPot.x;
    point.y = slot.block_GroundPt.BlockPot.y;
    point.z = 0.0;
    marker.points.push_back(std::move(point));
    marker.colors.push_back(std::move(color));

    markers.markers.emplace_back(marker);
}

void PseNode::PushSinglePldEdgeMarker(const PLD_DL_Result& slot, const uint16_t id,
                                      visualization_msgs::msg::MarkerArray& markers)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "edge";
    marker.id = id;
    marker.type = 8;
    marker.action = 0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    std_msgs::msg::ColorRGBA color;
    color.r = 0;
    color.g = 1;
    color.b = 1;
    color.a = 1;
    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    for (uint32_t i = 0U ; i < 2 ; ++i) {
        geometry_msgs::msg::Point point;
        if (slot.edge_GroundPt[i].EdgePot_type == 0) {
            continue;
        }
        point.x = slot.edge_GroundPt[i].EdgePot.x;
        point.y = slot.edge_GroundPt[i].EdgePot.y;
        point.z = 0.0;
        marker.points.push_back(std::move(point));
        marker.colors.push_back(std::move(color));
    }
    markers.markers.emplace_back(marker);
}

void PseNode::ShowVisionFreespacePoint(const AvpeFreespace::Ptr& avpeMsg, visualization_msgs::msg::MarkerArray& markers)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "freespace";
    marker.type = 8;
    marker.action = 0;

    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;

    std_msgs::msg::ColorRGBA color;
    color.r = 1;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    {
        // vehicle type
        marker.id = 5;
        color.g = 0.5;
        for (const auto& pt : avpeMsg->points) {
            geometry_msgs::msg::Point point;
            if (pt.type == 5) {
                point.x = pt.point.x();
                point.y = pt.point.y();
                point.z = 0.0;
                marker.points.push_back(point);
                marker.colors.push_back(color);
            }
        }
        markers.markers.push_back(marker);
    }

    {
        // unvehicle type
        marker.points.clear();
        marker.colors.clear();
        marker.id = 0;
        color.g = 0;
        for (const auto& pt : avpeMsg->points) {
            geometry_msgs::msg::Point point;
            if (pt.type != 5 && pt.type != 0) {
                point.x = pt.point.x();
                point.y = pt.point.y();
                point.z = 0.0;
                marker.points.push_back(std::move(point));
                marker.colors.push_back(std::move(color));
            }
        }
        markers.markers.push_back(marker);
    }

    perAvpeMarkerPub_->publish(markers);
}

void PseNode::ShowFusionStaticElements(const StaticElements& elements)
{
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "static_element";
    marker.type = 4;
    marker.action = 0;

    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;

    std_msgs::msg::ColorRGBA color;
    color.r = 1;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    marker.id = 0;
    for (const auto& ele : elements.elems) {
        marker.points.clear();
        marker.colors.clear();
        for (const auto& vertice : ele.vertices_list) {
            geometry_msgs::msg::Point point;
            point.x = vertice.x;
            point.y = vertice.y;
            point.z = 0.0;

            marker.points.push_back(point);
            marker.colors.push_back(color);
        }
        ++marker.id;
        markers.markers.push_back(marker);
    }

    fusStaticElePub_->publish(markers);
}


void PseNode::PushSegMarker(const AvpeSegmentation& avpSeg,
                            const uint16_t id,
                            visualization_msgs::msg::MarkerArray& markers)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "seg";
    marker.id = id;
    marker.type = 8;
    marker.action = 0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    std_msgs::msg::ColorRGBA color;
    color.r = 0;
    color.g = 0;
    color.b = 1;
    color.a = 1;
    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);


    for (uint32_t i = 1U ; i < avpSeg.region.size() ; ++i) {
        geometry_msgs::msg::Point point;
        point.x = avpSeg.region[i].point.x();
        point.y = avpSeg.region[i].point.y();
        point.z = 0.0;
        marker.points.emplace_back(point);
        marker.colors.emplace_back(color);
    }
    markers.markers.emplace_back(marker);
}

#endif


}
}