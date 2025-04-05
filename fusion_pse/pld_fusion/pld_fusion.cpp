#include "pld_fusion/pld_fusion.h"

#include "utils/matrix_utils.h"

#include <iostream>

namespace Fusion {
namespace PSE {

void PldFusion::CANCallBack(const vehicle_msgs::msg::CanCopyCp2Ap50msStruct::SharedPtr msg)
{
    if( (0 == (int)msg->icc50mssiggwpdu01.idrvrdooropensts_chcanfd)
     && (0 == (int)msg->icc50mssiggwpdu01.ifrtpsngdooropensts_chcanfd)
     && (0 == (int)msg->icc50mssiggwpdu01.ildspcopensts)
     && (0 == (int)msg->icc50mssiggwpdu01.irldooropensts_chcanfd)
     && (0 == (int)msg->icc50mssiggwpdu01.irrdooropensts_chcanfd) )
    {
        isDoorStatusOk_ = true;
    }
    else
    {
        isDoorStatusOk_ = false;
    }

}

void PldFusion::ParkSmCallback(const mpc_prk_msgs::msg::MsgSmHmi2Pnc::SharedPtr msg)
{
    isParking_ = (msg->hmi_pnc_info.parkin_slot_id != 0) 
              && ( ((msg->sm_pnc_info.apa_interior_status >= 9) && (msg->sm_pnc_info.apa_interior_status <= 12))
                || ((msg->sm_pnc_info.rpastsin >= 5) && (msg->sm_pnc_info.rpastsin <= 12)) );
    if(isParking_) {
        mergedSlots_.targetSlotId = (uint32_t)msg->hmi_pnc_info.parkin_slot_id;
    }
    else {
        mergedSlots_.targetSlotId = 0;
        mergedSlots_.targetSlotLoc = FusionSlot::SlotLoc::UNKNOWN;
    }
}

bool PldFusion::MakeFrame(PldFrame& frame)
{
    return manager_.MakeFrame(frame);
}

void PldFusion::Detect(PldFrame& frame)
{
    // if(Fusion::isZero(frame.poseOb.translation[0]) 
    // && Fusion::isZero(frame.poseOb.translation[1])
    // && Fusion::isZero(frame.poseOb.translation[2]) ) {
    //     frame.detectSlots_.clear();
    //     return;
    // }

    isUpdate_ = (frame.poseOb.translation - lastOdom_.translation).norm() > -0.1;
    isMoving_ = frame.poseOb.linearVelocity.norm() > 0.3;   //speed larger than 0.1 m/s
    //std::cout<<"speed: "<<frame.poseOb.linearVelocity.norm()<<"\n";

    if ( (frame.pld_ != nullptr) && (isDoorStatusOk_) ) {
        ObserveRange->TransferToWorld(frame.poseOb);
        ObserveRange->initLines();
        frame.detectSlots_ = GenerateFusionSlot(frame);

        std::cout<<"dist:";

        for(int i = 0; i < frame.detectSlots_.size(); i++) {
            for(int j = 0; j < 4; j++) {
                PldFusion::isCornerAsTrunctionPoint(
                    imageRange,frame.detectSlots_[i]->corners[j]);
            }
        }

        std::cout<<"\n";
    }

    if(frame.uss_ != nullptr) {
        ussSlotInputProcess(ussSlots_, frame, isParking_);
    }

    for(auto iter = ussSlots_.slotsList_.begin(); iter != ussSlots_.slotsList_.end();) {
        if( (PldFusion::slotAdjustUsingFs(*iter, frame.fs_) == false)
         && (isParking_ == false)) {
            if(debugParameter.isUSSDeleteOpen == true) {
                ussSlots_.deleteSlot(iter);
            }
        }
        else {
            ++iter;
        }
    }

}

void PldFusion::ussSlotInputProcess(CUssSlotsList& ussSlots_, PldFrame& frame, const bool isInParkIn)
{
    
    // std::cout<<"leftUpdateValue: "<<(int)ussSlots_.leftUpdateValue<<" rightUpdateValue: "<<(int)ussSlots_.rightUpdateValue<<"\n";
    // std::cout<<"inValidLeftUpdateValue: "<<(int)ussSlots_.inValidLeftUpdateValue<<" inValidRightUpdateValue: "<<(int)ussSlots_.inValidRightUpdateValue<<"\n";


    // std::cout<<"input left: "<<(bool)frame.uss_->leftSlot->isSlotInfoValid<<", "<<(int)frame.uss_->leftSlot->updateValue;
    // std::cout<<" input right: "<<(bool)frame.uss_->rightSlot->isSlotInfoValid<<", "<<(int)frame.uss_->rightSlot->updateValue<<"\n";

    // std::cout<<"isLastFrameInParking: "<<ussSlots_.isLastFrameInParking<<"\n";
    // std::cout<<"isInParkIn: "<<isInParkIn<<"\n";
    // std::cout<<"num of uss slot: "<<ussSlots_.slotsList_.size()<<"\n";

    if( (ussSlots_.isLastFrameInParking == true)
     && (isInParkIn == false) )
    {
        ussSlots_.reset();
    }
    ussSlots_.isLastFrameInParking = isInParkIn;



    if(frame.uss_->leftSlot->isSlotInfoValid == false) {
        ussSlots_.inValidLeftUpdateValue = 16U;
    }

    if(frame.uss_->rightSlot->isSlotInfoValid == false) {
        ussSlots_.inValidRightUpdateValue = 16U;
    }

    if(isInParkIn == true) {
        frame.uss_ = nullptr;
        return;
    }

    FusionSlot::Ptr newLeftSlot(nullptr);
    FusionSlot::Ptr newRightSlot(nullptr);

    if(frame.uss_->leftSlot->isSlotInfoValid) {

        newLeftSlot = std::make_shared<FusionSlot>(frame.uss_->leftSlot, frame.poseOb);
        if(ussSlots_.leftUpdateValue == frame.uss_->leftSlot->updateValue) {
            if(ussSlots_.currentLeftSlot != nullptr) {
                //std::cout<<"left switch 1"<<"\n";
                ussSlots_.replace(ussSlots_.currentLeftSlot, newLeftSlot);
            }
            else {
                //std::cout<<"left switch 2"<<"\n";
                //do nothing
            }
        }
        else {
            if(!IsConflictWithFSAndPld(newLeftSlot, frame.fs_, trackSlots_.slotsList_)) {
                //std::cout<<"left switch 3"<<"\n";
                ussSlots_.Push_back(newLeftSlot, frame.uss_->leftSlot->updateValue);
            }
            else {
                //std::cout<<"left switch 4"<<"\n";
                //std::cout<<"input delete"<<"\n";
                ussSlots_.leftUpdateValue = frame.uss_->leftSlot->updateValue;
                ussSlots_.currentLeftSlot = nullptr;
            }
        }
    }
    


    if(frame.uss_->rightSlot->isSlotInfoValid) {

        newRightSlot = std::make_shared<FusionSlot>(frame.uss_->rightSlot, frame.poseOb);
        if(ussSlots_.rightUpdateValue == frame.uss_->rightSlot->updateValue) {
            if(ussSlots_.currentRightSlot != nullptr) {
                //std::cout<<"right switch 1"<<"\n";
                ussSlots_.replace(ussSlots_.currentRightSlot, newRightSlot);
            }
            else {
                //std::cout<<"right switch 2"<<"\n";
                //do nothing
            }
        }
        else {
            if(!IsConflictWithFSAndPld(newRightSlot, frame.fs_, trackSlots_.slotsList_)) {
                //std::cout<<"right switch 3"<<"\n";
                ussSlots_.Push_back(newRightSlot, frame.uss_->rightSlot->updateValue);
            }
            else {
                //std::cout<<"right switch 4"<<"\n";
                //std::cout<<"input delete"<<"\n";
                ussSlots_.rightUpdateValue = frame.uss_->rightSlot->updateValue;
                ussSlots_.currentRightSlot = nullptr;
            }
        }
    }

    ussSlots_.deleteSlot(frame.poseOb);
    if(frame.fs_ !=nullptr) {
        for(auto iter = ussSlots_.slotsList_.begin(); iter != ussSlots_.slotsList_.end();) {
            if(IsConflictWithFSAndPld(*iter, frame.fs_, trackSlots_.slotsList_)) {
                ussSlots_.deleteSlot(iter);
            }
            else {
                ++iter;
            }
        }
    }

    frame.uss_ = nullptr;
}

bool PldFusion::IsConflictWithFSAndPld(
    const FusionSlot::Ptr slot, 
    const FSStruct::Ptr fsStruct, 
    const std::list<FusionSlot::Ptr> pldSlots)
{
    if(fsStruct == nullptr) {
        return false;
    }

    float schrumpfRadio = 0.6f;
    FusionSlot::Ptr coreSlotPtr = std::make_shared<FusionSlot>();
    for(int i = 0; i < 4; i++) {
        coreSlotPtr->corners[i].worldCorner = schrumpfRadio*(slot->corners[i].worldCorner - slot->middlePoint.worldCorner) + slot->middlePoint.worldCorner;
    }

    float P0Tau = slot->originP0Tau;     //lines_[0]->getStartPointTau();
    float P1Tau = slot->originP1Tau;     //lines_[0]->getEndPointTau();
    float tauThreshold0 = (double)(P1Tau - P0Tau)/4.0 + P0Tau;
    float tauThreshold1 = (double)(P1Tau - P0Tau)*3.0/4.0 + P0Tau;

    float minDepth = (slot->type == FusionSlot::Type::HORIZONTAL)?1.5f:3.5f;
    float minWidth = (slot->type == FusionSlot::Type::HORIZONTAL)?5.2f:3.0f;

    coreSlotPtr->initLines();
    Eigen::Vector3d middlePoint;
    middlePoint << slot->middlePoint.worldCorner[0], slot->middlePoint.worldCorner[1], 0.0;
    //std::cout<<"size of outerObj: "<<fsStruct->outerObjects_.size()<<"\n";
    //std::cout<<"middle point: "<<slot->middlePoint.worldCorner[0]<<","<<slot->middlePoint.worldCorner[1]<<","<<slot->middlePoint.worldCorner[2]<<"\n";
    for(int i = 0; i <  fsStruct->outerObjects_.size(); i++) {
        for(int j = 0; j < fsStruct->outerObjects_[i].size(); j++) {
            if((middlePoint - fsStruct->outerObjects_[i][j].worldCorner).norm() > 8.0) {
                continue;
            }
            else {
                if( (isPointInsideSlot(coreSlotPtr->lines_, fsStruct->outerObjects_[i][j].worldCorner))
                 || ((slot->lines_[0]->calcOrthDistOfWorldPointAbs(fsStruct->outerObjects_[i][j].worldCorner) < minDepth)
                  && Fusion::isValuesInOrder(
                        tauThreshold0,
                        slot->lines_[0]->calcTauOfOrthProj(fsStruct->outerObjects_[i][j].worldCorner),
                        tauThreshold1)) ) {
                    //std::cout<<"conflict"<<"\n";
                    return true;
                }
            }
        }
    }

    for(int i = 0; i <  fsStruct->innerObjects_.size(); i++) {
        for(int j = 0; j < fsStruct->innerObjects_[i].size(); j++) {
            if((middlePoint - fsStruct->innerObjects_[i][j].worldCorner).norm() > 8.0) {
                continue;
            }
            else {
                if( (isPointInsideSlot(coreSlotPtr->lines_, fsStruct->innerObjects_[i][j].worldCorner))
                 || ((slot->lines_[0]->calcOrthDistOfWorldPointAbs(fsStruct->innerObjects_[i][j].worldCorner) < minDepth)
                  && Fusion::isValuesInOrder(
                        tauThreshold0,
                        slot->lines_[0]->calcTauOfOrthProj(fsStruct->innerObjects_[i][j].worldCorner),
                        tauThreshold1)) ) {
                    //std::cout<<"conflict"<<"\n";
                    return true;
                }
            }
        }
    }

    for(auto& pldSlot : pldSlots) {
        pldSlot->initLines();

        Fusion::CTrackLine::Ptr ussOpenLine = slot->lines_[0];
        Fusion::CTrackLine::Ptr pldOpenLine = pldSlot->lines_[0];

        float l_angle_r = Fusion::calAngleBetweenTwoUnitVector(
            ussOpenLine->getDefaultUnitVec(),
            pldOpenLine->getDefaultUnitVec());

        if((float)35.0f*M_PI/180.0f > std::fabs(std::fabs(l_angle_r) - M_PI/2.0f)) {
            if( (2.0f > Fusion::calcDistBetweenPoints(ussOpenLine->getStartPoint(), pldOpenLine->getStartPoint()))
            ||  (2.0f > Fusion::calcDistBetweenPoints(ussOpenLine->getStartPoint(), pldOpenLine->getEndPoint()))
            ||  (2.0f > Fusion::calcDistBetweenPoints(ussOpenLine->getEndPoint(), pldOpenLine->getStartPoint()))
            ||  (2.0f > Fusion::calcDistBetweenPoints(ussOpenLine->getEndPoint(), pldOpenLine->getEndPoint())) ) {
                return true;
            }
        }
    }

    return false;
}

bool PldFusion::slotAdjustUsingFs(const FusionSlot::Ptr slot, const FSStruct::Ptr fsStruct)
{
    if(fsStruct == nullptr) {
        return true;
    }
    
    if(slot->lines_[0] == nullptr) {
        slot->initLines();
    }

    slot->recoverOriginUssSlot();

    float minDepth = (slot->type == FusionSlot::Type::HORIZONTAL)?1.5f:3.5f;
    float minWidth = (slot->type == FusionSlot::Type::HORIZONTAL)?5.2f:2.5f;

    float tau0Middle = slot->originP0Tau - slot->middleTau; 
    float tauOut0 = (tau0Middle > 0)?(tau0Middle + 5):(tau0Middle - 5);

    float tau1Middle = slot->originP1Tau - slot->middleTau; 
    float tauOut1 = (tau1Middle > 0)?(tau1Middle + 5):(tau1Middle - 5);
    
    Eigen::Vector3d middlePoint;
    middlePoint << slot->middlePoint.worldCorner[0], slot->middlePoint.worldCorner[1], 0.0;
    //std::cout<<"size of outerObj: "<<fsStruct->outerObjects_.size()<<"\n";
    //std::cout<<"middle point: "<<slot->middlePoint.worldCorner[0]<<","<<slot->middlePoint.worldCorner[1]<<","<<slot->middlePoint.worldCorner[2]<<"\n";
    for(int i = 0; i <  fsStruct->outerObjects_.size(); i++) {
        for(int j = 0; j < fsStruct->outerObjects_[i].size(); j++) {
            if((middlePoint - fsStruct->outerObjects_[i][j].worldCorner).norm() > 8.0) {
                continue;
            }
            else {
                bool returnValue = PldFusion::processWithFSPoint(
                    minDepth,
                    tau0Middle,
                    tau1Middle,
                    tauOut0,
                    tauOut1,
                    slot,
                    fsStruct->outerObjects_[i][j].worldCorner);
                if(returnValue) {
                    return false;
                }
            }
        }
    }

    for(int i = 0; i <  fsStruct->innerObjects_.size(); i++) {
        for(int j = 0; j < fsStruct->innerObjects_[i].size(); j++) {
            if((middlePoint - fsStruct->innerObjects_[i][j].worldCorner).norm() > 8.0) {
                continue;
            }
            else {
                bool returnValue = PldFusion::processWithFSPoint(
                    minDepth,
                    tau0Middle,
                    tau1Middle,
                    tauOut0,
                    tauOut1,
                    slot,
                    fsStruct->innerObjects_[i][j].worldCorner);
                if(returnValue) {
                    return false;
                }
            }
        }
    }
    
    if(std::fabs(tauOut0 - tauOut1) < minWidth) {
        return false;
    }

    float differTau0 = std::fabs(tauOut0 - tau0Middle);
    bool isP0WithInvade = false;
    if(std::fabs(tauOut0) < std::fabs(tau0Middle)) {
        //with invade
        isP0WithInvade = true;
    }

    float differTau1 = std::fabs(tauOut1 - tau1Middle);
    bool isP1WithInvade = false;
    if(std::fabs(tauOut1) < std::fabs(tau1Middle)) {
        //with invade
        isP1WithInvade = true;
    }

    tauOut0 = tauOut0 + slot->middleTau;
    tauOut1 = tauOut1 + slot->middleTau;

    Eigen::Vector3d newP0 = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
    Eigen::Vector3d newP1 = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
    if((isP0WithInvade == true)
    && (isP1WithInvade == true)) {
        newP0 = slot->lines_[0]->calcWorldPointUsingTauValue(tauOut0);
        newP1 = slot->lines_[0]->calcWorldPointUsingTauValue(tauOut1);
    }
    else if((isP0WithInvade == true)
         && (isP1WithInvade == false)) {
        newP0 = slot->lines_[0]->calcWorldPointUsingTauValue(tauOut0);
        if(differTau0 > differTau1) {
            newP1 = slot->lines_[0]->calcWorldPointUsingTauValue(tauOut1);
        }
        else {
            newP1 = newP0 + slot->lines_[0]->getDefaultUnitVec()*slot->lines_[0]->getLength();
        }
    }
    else if((isP0WithInvade == false)
         && (isP1WithInvade == true)) {
        newP1 = slot->lines_[0]->calcWorldPointUsingTauValue(tauOut1);
        if(differTau1 > differTau0) {
            newP0 = slot->lines_[0]->calcWorldPointUsingTauValue(tauOut0);
        }
        else {
            newP0 = newP1 - slot->lines_[0]->getDefaultUnitVec()*slot->lines_[0]->getLength();
        }     
    }
    else {
        newP0 = slot->lines_[0]->calcWorldPointUsingTauValue(slot->originP0Tau);
        newP1 = slot->lines_[0]->calcWorldPointUsingTauValue(slot->originP1Tau);
    }

    slot->corners[0].worldCorner = newP0;
    slot->corners[1].worldCorner = newP1;

    Eigen::Vector3d depthUV = 
        Fusion::turnUnitVectorFor90DegreeInAntiClockwiseInVehicleCoordinate(slot->lines_[0]->getDefaultUnitVec());
    slot->corners[2].worldCorner = newP1 +  depthUV*slot->lines_[1]->getLength();
    slot->corners[3].worldCorner = newP0 +  depthUV*slot->lines_[3]->getLength();

    return true;


}


bool PldFusion::processWithFSPoint(
    const float minDepth,
    const float tau0Middle,
    const float tau1Middle,
    float& tauOut0,
    float& tauOut1,
    const FusionSlot::Ptr slot,
    const Eigen::Vector3d ObjworldPoint)
{
    if( slot->lines_[0]->isPointOnLeftSideOfUnitVector(ObjworldPoint)
    && (slot->lines_[0]->calcOrthDistOfWorldPointAbs(ObjworldPoint) < minDepth) ) {
    
        float tauDifferWithMiddleTau = 
            slot->lines_[0]->calcTauOfOrthProj(ObjworldPoint) - slot->middleTau;
        
        if(tau0Middle*tauDifferWithMiddleTau > 0) {
            tauOut0 = 
                (tauOut0 > 0)?std::min(tauDifferWithMiddleTau, tauOut0):std::max(tauDifferWithMiddleTau, tauOut0);
        }
        else if(tau1Middle*tauDifferWithMiddleTau > 0) {
            tauOut1 = 
                (tauOut1 > 0)?std::min(tauDifferWithMiddleTau, tauOut1):std::max(tauDifferWithMiddleTau, tauOut1);         
        }
        else {
            //==0
            return true;
        }
    }

    return false;

}


std::vector<FusionSlot::Ptr> PldFusion::GenerateFusionSlot(const PldFrame& frame)
{
    std::vector<FusionSlot::Ptr> detectSlots;
    const auto& perceptSlot = frame.pld_;
    for (uint16_t i = 0U; i < perceptSlot->frame.slot_num; ++i) {
        const auto& slot = perceptSlot->frame.DL_Result_one_frame[i];
        FusionSlot::Ptr fusionSlot = std::make_shared<FusionSlot>(slot, frame.poseOb);
        FusionSlot::Ptr fusionSlotBak = std::make_shared<FusionSlot>(*fusionSlot);
        fusionSlot->relativeSlots.Push(fusionSlotBak);

        if ((fusionSlot->finalStatus == FusionSlot::Status::OCCUPIED) && frame.vsFs_ != nullptr && !frame.vsFs_->worldPoints.empty()) {
            AlignSlotWithFreespaceType(fusionSlot, frame.vsFs_);
        }
        
        if ((fusionSlot->finalStatus == FusionSlot::Status::OCCUPIED) && (frame.vsObject_ != nullptr)) {
            AlignSlotWithAvpe(fusionSlot, frame.vsObject_);
        }
        detectSlots.push_back(fusionSlot);
    }
    return detectSlots;
}

void PldFusion::AlignSlotWithFreespaceType(const FusionSlot::Ptr& slot, const VisionFreespace::Ptr& vsFs)
{
    for (size_t i = 0; i < vsFs->worldPoints.size(); ++i) {
        if (vsFs->inputFs->points[i].type == 0) {
            continue;
        }
        const auto& pt = vsFs->worldPoints[i];
        bool isInSlot = true;
        uint8_t index = 0U;
        while (index < 4U && isInSlot) {
            const Eigen::Vector2d vEdge = (slot->corners[(index + 1U) % 4U].worldCorner - slot->corners[index].worldCorner).segment<2U>(0U);
            const Eigen::Vector2d vPoint = pt - slot->corners[index].worldCorner.segment<2U>(0U);
            isInSlot &= (vEdge.dot(vPoint) > 0.0);
            ++index;
        }
        if (isInSlot) {
            if (vsFs->inputFs->points[i].type == 5) {
                slot->fsVehPointTypes.push_back(vsFs->inputFs->points[i].type);
            } else {
                slot->fsOtherPointTypes.push_back(vsFs->inputFs->points[i].type);
            }
        }
    }
}

void PldFusion::AlignSlotWithAvpe(
    const FusionSlot::Ptr& slot,
    const visionObjVector::Ptr& vsObject)
{
    slot->initLines();
    for(const auto& lockerObj : vsObject->vsObjVector) {
        if(true == Fusion::isPointInsideSlot(slot->lines_, lockerObj->centralPosition)) {
            slot->isOccupiedByLocker = true;
            return;
        }
    }
}

void PldFusion::Track(PldFrame& frame)
{
    if(isDoorStatusOk_ == false)
    {
        return;
    }

    std::vector<FusionSlot::Ptr> noMatchSlots;

    PldFusion::matchAlgo(frame, noMatchSlots);
    
    for(auto& tSlot : trackSlots_.slotsList_) {
        FusionSlot::Ptr matchedNewSlot = tSlot->lowestCostInfo.matchedSlot;
        // if((tSlot->id == 5) || (tSlot->id == 4)) {
        //     std::cout<<"slot id: "<<tSlot->id<<" num of matched info: "<<tSlot->matchedList.size()<<" is matched: "<<(matchedNewSlot == nullptr)<<" is confirmed: "<<tSlot->isConfirm<<"\n";
        // }

        if(matchedNewSlot != nullptr) {
            int pointsIndexDiff(0);
    
            for(int i = 0; i < 4; i++)
            {
                bool isMatchedLinesFound(false);
                for(int j = 0; j < 4; j++)
                {
                    float l_errorOfOrthDist_f(0.0f);
                    float l_minOverlapRatio_f(0.0f);
                    if(true == PldFusion::isLinesMatched(
                        tSlot->lines_[j], 
                        matchedNewSlot->lines_[i], 
                        l_errorOfOrthDist_f, 
                        l_minOverlapRatio_f))
                    {
                        //get step from index j to index i in anti-clockwise direction
                        pointsIndexDiff = (j >= i)?(j - i): (j + 4 - i);
                        isMatchedLinesFound = true;
                        break;
                    }
                }
                if(isMatchedLinesFound == true) {
                    break;
                }
            }

            matchedNewSlot->CornersAntiClockwiseRotate(pointsIndexDiff);
            matchedNewSlot->alignTrackSlot = tSlot;
            CalculatePerceptErrorToCenter(matchedNewSlot, tSlot);

            if (isUpdate_) {
                matchedNewSlot->calcCornerCost(
                    debugParameter.baseCostRatio, 
                    tSlot->type,
                    debugParameter.addBuffer,
                    debugParameter.minBasePointUpdateCost,
                    debugParameter.maxBasePointUpdateCost,
                    debugParameter.maxBuffer,
                    debugParameter.addBufferXThreshold,
                    debugParameter.depthCornerRatio,
                    frame.poseOb);
                tSlot->relativeSlots.Push(matchedNewSlot);
                // if((tSlot->id == debugParameter.debugSlotId)||(tSlot->id == debugParameter.debugSlotId2)) {
                //     std::cout<<"slot id: "<<tSlot->id<<" matched & index diff: "<<pointsIndexDiff<<"\n";
                //     for(int i = 0; i < 4; i++) {
                //         std::cout<<"corner "<<i<<": "<<tSlot->corners[i].baseCorner[0]<<" ,"<<tSlot->corners[i].baseCorner[1]
                //             <<"   new corner "<<i<<": "<<matchedNewSlot->corners[i].baseCorner[0]<<" ,"<<matchedNewSlot->corners[i].baseCorner[1]<<"\n";
                //     }
                // }
                matchedNewSlot->indexDiffWithAlignTrackSlotInAntiClockwise = pointsIndexDiff;
                tSlot->updateActive = true;
            }
        }
    }




    
    if (isUpdate_) {

        FusionMatchSlots(frame, trackSlots_.slotsList_);
        lastOdom_ = frame.poseOb;
    }

    if (!isParking_) {
        GenerateNewTrackSlots(noMatchSlots, trackSlots_);
        if(isUpdate_) {
            trackSlots_.deleteSlot(frame.poseOb);
        }
    }

    if (!isParking_) {
        PldFusion::openLineCalc(trackSlots_.slotsList_, frame);
    }

    //trackSlots_.resetEachCircleForPLDTracking();
    #if DEBUG_MODE
    //ShowFusionPld(trackSlots_.slotsList_);
    #endif
}

void PldFusion::checkCornerConfidenceMatchedSlot(const FusionSlot::Ptr& fusionSlot)
{
    FusionSlot::Ptr perceptSlot = fusionSlot->relativeSlots.back();
    for(int i = 0; i < 4; i++) {
        if(perceptSlot->corners[i].confidence < 0.3) {
            perceptSlot->corners[i].confidence = 0.0f;
        }
    }

    if(fusionSlot->isParkable == false) {
        return;
    }

    for(int i = 0; i < 4; i++) {
        if(perceptSlot->corners[i].weightForBaseCorner > debugParameter.maxBasePointUpdateCost) 
        {
            perceptSlot->corners[i].confidence = 0.0f;
            continue;
        }

        // if(fusionSlot->id == 2) {
        //     std::cout<<"is percept parallel: "
        //     <<(fusionSlot->type == FusionSlot::Type::HORIZONTAL)<<" is corner "<<i<<" inside: "
        //     <<isPointInsideSlot(vehicleRange->lines_, perceptSlot->corners[i].baseCorner)<<"\n";
        // }
        if((fusionSlot->type == FusionSlot::Type::HORIZONTAL)
        && (isPointInsideSlot(vehicleRange->lines_, perceptSlot->corners[i].baseCorner) == true)) {
            perceptSlot->corners[i].confidence = 0.0f;
        }
    }
}

void PldFusion::CalculatePerceptErrorToCenter(const FusionSlot::Ptr& perSlot, const FusionSlot::Ptr& trackSlot)
{
    for (int i = 0; i < 4; ++i) {
        perSlot->corners[i].covError.error = std::min((perSlot->corners[i].worldCorner - trackSlot->corners[i].worldCorner).norm(), 2.5);
        perSlot->corners[i].covError.error = debugParameter.covErrorRatio * perSlot->corners[i].covError.error;
    }
}

void PldFusion::FusionMatchSlots(PldFrame& frame, std::list<FusionSlot::Ptr>& trackSlots)
{
    

    static constexpr float maxLifeCount = 20;
    for (auto& tSlot : trackSlots) {
        tSlot->setIsParkable(debugParameter, frame.poseOb);
        if (!isParking_) {
            if(tSlot->updateActive) {
                tSlot->lifeCounter++;
            }
            else {
                if(isMoving_ 
                && PldFusion::isInsideObserveRange(ObserveRange, tSlot) ) {
                    tSlot->lifeCounter -= 0.5f;
                }
            }
            //tSlot->lifeCounter = tSlot->updateActive ? (tSlot->lifeCounter + 1) : (tSlot->lifeCounter - 1);
        }
        if (tSlot->updateActive) {

            tSlot->lifeCounter = std::min(tSlot->lifeCounter,maxLifeCount);
            tSlot->isConfirm = tSlot->lifeCounter == maxLifeCount ? true : tSlot->isConfirm;
            
            //average confidence of percept slots
            
            // if((tSlot->isParkable == true) 
            // && ((tSlot->id == debugParameter.debugSlotId)||(tSlot->id == debugParameter.debugSlotId2)) )
            //   {
            //     std::cout<<"slot id: "<<tSlot->id<<"\n";
            // }

            PldFusion::checkCornerConfidenceMatchedSlot(tSlot);
            tSlot->confidence_ = CalculateWeightOfPerceptSlots(tSlot);

            // if((tSlot->id == debugParameter.debugSlotId)||(tSlot->id == debugParameter.debugSlotId2)) {
            //     std::cout<<"slot id: "<<tSlot->id;
            // }

            for (int i = 0; i < 4; ++i) {
                double updateScale = tSlot->confidence_ * tSlot->corners[i].covError.weight;
                Eigen::Vector3d wordCornerTmp(0.0, 0.0, 0.0);
                for (const auto& relative : tSlot->relativeSlots) {
                    wordCornerTmp += (relative->corners[i].covError.weight * relative->corners[i].worldCorner);
                }
                // if((tSlot->id == debugParameter.debugSlotId)||(tSlot->id == debugParameter.debugSlotId2)) {
                //     std::cout<<"corner "<<i<<" update scale before: "<<updateScale
                //             <<" wordCornerTmp:"<<wordCornerTmp[0]<<","<<wordCornerTmp[1]<<" ,"<<wordCornerTmp[2]<<"\n";
                // }
                updateScale = ((updateScale < 1e-5) || (wordCornerTmp.norm() < 1e-5)) ? 0.0f : updateScale;

                // if( (tSlot->isParkable == true)
                //  && ((tSlot->id == debugParameter.debugSlotId)||(tSlot->id == debugParameter.debugSlotId2))) {

                // // if((tSlot->id == debugParameter.debugSlotId)||(tSlot->id == debugParameter.debugSlotId2)) {
                // //     std::cout<<"corner "<<i<<" update scale: "<<updateScale<<"\n";
                // // }
                // std::cout<<"corner "<<i;
                // if(updateScale > 0.001) {
                //     std::cout<<" updating    ";
                // }
                // else {
                //    std::cout<<" not updating    "; 
                // }
                // }
                tSlot->corners[i].worldCorner = (1.0 - updateScale) * tSlot->corners[i].worldCorner + updateScale  * wordCornerTmp;
            }
            // if((tSlot->isParkable == true)
            // && ((tSlot->id == debugParameter.debugSlotId)||(tSlot->id == debugParameter.debugSlotId2))) {
            //     std::cout<<"\n";
            // }
            CalculateCovarianceOfTrackSlots(tSlot);
            FusionBlockPoint(frame, tSlot);
            if(!isParking_)
            {
                EstimatePldStatus(tSlot);
            }

            EstimateOccupiedType(tSlot);
            // if(!isParking_)
            // {
            //     EstimateOpenLinePosition(tSlot);
            // }

            tSlot->TransferToBase(frame.poseOb);
            tSlot->calcMiddlePoint(frame.poseOb);

            // for(int i = 0; i < 4; i++) {
            //     if((tSlot->id == debugParameter.debugSlotId)||(tSlot->id == debugParameter.debugSlotId2)) {
            //         std::cout<<"slot id:"<<tSlot->id<<" fs corner "<<i<<": "<<tSlot->corners[i].baseCorner[0]<<", "<<tSlot->corners[i].baseCorner[1]<<"\n";
            //     }
            // }

            tSlot->updateActive = false;

            for(int i = 0; i < 4; i++) {
                //PldFusion::isCornerAsTrunctionPoint(imageRange, tSlot->relativeSlots.back() corners[i]);
                PldFusion::isCornerAsTrunctionPoint(imageRange, tSlot->corners[i]);

                if( (tSlot->relativeSlots.back()->corners[i].isCornerAsTruePoint == true) 
                 && ( (tSlot->corners[i].isCornerAsTruePoint == true)
                   || ((tSlot->corners[i].worldCorner - tSlot->relativeSlots.back()->corners[i].worldCorner).norm() < 1.0f)) ){
                    //as true point
                    tSlot->corners[i].isCornerAsTruePoint = true;
                }
                else {
                    tSlot->corners[i].isCornerAsTruePoint = false;
                }

                if(tSlot->id == 1) {
                    std::cout<<"corner "<<i<<": "<<"as true point: "<<tSlot->corners[i].isCornerAsTruePoint
                             <<" input as true point: "<<tSlot->relativeSlots.back()->corners[i].isCornerAsTruePoint<<"\n";

                }

            }
        }
    }
}

double PldFusion::CalculateWeightOfPerceptSlots(const FusionSlot::Ptr& tSlot)
{
    for (int i = 0; i < 4; ++i) {
        double weightSum = 0.0;
        //float cornerCostSum = 0.0f;
        //float sumX = 0.0f;
        for (auto& perceptSlot : tSlot->relativeSlots) {

            // if ( (perceptSlot->corners[i].confidence < 0.3) 
            //   || ( (perceptSlot->corners[i].weightForBaseCorner > debugParameter.maxBasePointUpdateCost)
            //     && (tSlot->isParkable == true) )) {
            //     perceptSlot->corners[i].confidence = 0.0;
            // }
            perceptSlot->corners[i].covError.weight = perceptSlot->corners[i].confidence * std::exp(-perceptSlot->corners[i].covError.error);
            // if(tSlot->isParkable == true) {
            //     std::cout<<perceptSlot->corners[i].covError.weight<<" ";
            // }
            weightSum += perceptSlot->corners[i].covError.weight;
        }

        // if( (tSlot->isParkable == true)
        //  && ((tSlot->id == debugParameter.debugSlotId)||(tSlot->id == debugParameter.debugSlotId2))) {
        //     cornerCostSum /= (float)tSlot->relativeSlots.size();
        //     sumX /= (float)tSlot->relativeSlots.size();
        //     // cornerCostSum = tSlot->corners[i].weightForBaseCorner;
        //     // sumX = tSlot->corners[i].baseCorner[0]; 
        //     //std::cout<<"\n"<<i<<" average cornerCost: "<<cornerCostSum<<" average base X: "<<sumX<<"\n";
        // }
        for (auto& perceptSlot : tSlot->relativeSlots) {
            //perceptSlot->corners[i].covError.weight = weightSum > 0.1 ? perceptSlot->corners[i].covError.weight / weightSum : 0.0;
            perceptSlot->corners[i].covError.weight = weightSum > 0.0001 ? perceptSlot->corners[i].covError.weight / weightSum : 0.0;
        }
        tSlot->corners[i].covError.weight = weightSum / tSlot->relativeSlots.size();
    }

    double confSum = 0.0;
    for (auto& perceptSlot : tSlot->relativeSlots) {
        confSum += perceptSlot->confidence_;
    }
    return confSum / static_cast<double>(tSlot->relativeSlots.size());
}

void PldFusion::CalculateCovarianceOfTrackSlots(FusionSlot::Ptr& trackSlot)
{
    for (int i = 0; i < 4; ++i) {
        //trackSlot->corners[i].covError.cov = 0.0;
        for (auto& perceptSlot : trackSlot->relativeSlots) {
            perceptSlot->corners[i].covError.error = std::min((perceptSlot->corners[i].worldCorner - trackSlot->corners[i].worldCorner).norm(), 2.5);
            perceptSlot->corners[i].covError.error = debugParameter.covErrorRatio * perceptSlot->corners[i].covError.error;

            //perceptSlot->corners[i].covError.error = (perceptSlot->corners[i].worldCorner - trackSlot->corners[i].worldCorner).norm();
            //trackSlot->corners[i].covError.cov += perceptSlot->corners[i].covError.weight * (perceptSlot->corners[i].covError.error * perceptSlot->corners[i].covError.error);
        }
    }
}

void PldFusion::EstimatePldStatus(FusionSlot::Ptr& trackSlot)
{
    std::array<uint16_t, 3U> statusCounter = {0};
    for (auto& perceptSlot : trackSlot->relativeSlots) {
        ++statusCounter[static_cast<int16_t>(perceptSlot->finalStatus)];
    }
    trackSlot->finalStatus = FusionSlot::Status::DEFAULT;
    uint16_t maxCount = statusCounter[0];

    for (int i = 1; i < 3; ++i) {
        if (statusCounter[i] > maxCount) {
            maxCount = statusCounter[i];
            trackSlot->finalStatus = static_cast<FusionSlot::Status>(i);
        }
    }
}

void PldFusion::EstimateOccupiedType(const FusionSlot::Ptr& trackSlot)
{
    if (trackSlot->finalStatus != FusionSlot::Status::OCCUPIED) {
        return;
    }
    std::array<size_t, 2U> statusCounter = {0};
    for (auto& perceptSlot : trackSlot->relativeSlots) {
        if (perceptSlot->finalStatus == FusionSlot::Status::OCCUPIED) {
            statusCounter[0] += perceptSlot->fsOtherPointTypes.size();
            statusCounter[1] += perceptSlot->fsVehPointTypes.size();
        }
    }
    if ((statusCounter[1] + statusCounter[0]) != 0) {
        trackSlot->isOccupiedByVeh = (statusCounter[1] >= statusCounter[0]);
    }

    if(trackSlot->isOccupiedByVeh == true) {
        return;
    }

    if(trackSlot->isLockerConfirmed == true) {
        trackSlot->isOccupiedByLocker = true;
        return;
    }


    std::array<size_t, 2U> groundLockerCounter = {0};
    
    int occupiedNum = 0;
    for(auto& perceptSlot : trackSlot->relativeSlots) {
        if (perceptSlot->finalStatus == FusionSlot::Status::OCCUPIED) {
            occupiedNum++;
            if(perceptSlot->isOccupiedByLocker == true) {
                groundLockerCounter[0]++;
            }
            else {
                groundLockerCounter[1]++;
            }
        }
    }
        
    //std::cout<<"slot id: "<<trackSlot->id<<" with locker: "<<groundLockerCounter[0]<<" without locker: "<<groundLockerCounter[1]<<"\n";
    if( (occupiedNum > 5)
     && ((float)groundLockerCounter[0] - (float)groundLockerCounter[1]) > -5.0f) {
        trackSlot->isOccupiedByLocker = true;
        if(groundLockerCounter[0] > groundLockerCounter[1]) {
            trackSlot->isLockerConfirmed = true;
        }
        //std::cout<<"occupied by locker set"<<"\n";
    }
    else {
        trackSlot->isOccupiedByLocker = false;
    }
    
}

void PldFusion::EstimateOpenLinePosition(const FusionSlot::Ptr& trackSlot)
{
    std::array<uint8_t, 4U> statusCounter = {0, 0, 0, 0};
    
    for (auto& perceptSlot : trackSlot->relativeSlots) {
        ++statusCounter[static_cast<uint8_t>(perceptSlot->indexDiffWithAlignTrackSlotInAntiClockwise)];
    }

    uint8_t indexDiff = 0;
    for (int i = 1; i < 4; i++) {
        if (statusCounter[i] > statusCounter[indexDiff]) {
            indexDiff = i;
        }
    }
    uint8_t rotateStep = 4 - indexDiff;
    
    
    trackSlot->CornersAntiClockwiseRotate(rotateStep);
    for(auto& perceptSlot : trackSlot->relativeSlots)
    {
        perceptSlot->CornersAntiClockwiseRotate(rotateStep);
        
        uint8_t indexDiffAfterRotate = perceptSlot->indexDiffWithAlignTrackSlotInAntiClockwise - indexDiff;
        perceptSlot->indexDiffWithAlignTrackSlotInAntiClockwise = (0 > (indexDiffAfterRotate))?(indexDiffAfterRotate + 4):indexDiffAfterRotate;
    }
}


bool PldFusion::isInsideObserveRange(const FusionSlot::Ptr ObserveRange, const FusionSlot::Ptr slot)
{
    return Fusion::isPointInsideSlot(ObserveRange->lines_, slot->middlePoint.worldCorner);
}

void PldFusion::FusionBlockPoint(PldFrame& frame, FusionSlot::Ptr& trackSlot)
{
    double weightSum = 0.0;
    for (uint16_t i = 0U; i < trackSlot->relativeSlots.size(); ++i) {
        auto& perceptSlot = trackSlot->relativeSlots.at(i);
        if (perceptSlot->block_.type == FusionSlot::BlockPoint::Type::NO_BLOCK) {
            perceptSlot->block_.confidence = 0.0;
        }
        perceptSlot->block_.confidence = std::max(0.0, perceptSlot->block_.confidence) * static_cast<double>(i + 1U);
        weightSum += perceptSlot->block_.confidence;
    }

    if (weightSum < 0.5) {
        if (trackSlot->block_.worldPoint.norm() < 1e-6) {
            trackSlot->block_.type = FusionSlot::BlockPoint::Type::NO_BLOCK;
            trackSlot->block_.worldPoint << 0.0, 0.0, 0.0;
        }
        else
        {
            if(true == PldFusion::isBlockInGoodObservationBlock(trackSlot, frame)) {
                trackSlot->block_.type = FusionSlot::BlockPoint::Type::NO_BLOCK;
                trackSlot->block_.worldPoint << 0.0, 0.0, 0.0;    
            }
        }
    } else {
        trackSlot->block_.type = FusionSlot::BlockPoint::Type::DETECTIVE;
        Eigen::Vector3d blockWorldPointTmp(0.0, 0.0, 0.0);   
        for (auto& perceptSlot : trackSlot->relativeSlots) {
            blockWorldPointTmp += ((perceptSlot->block_.confidence / weightSum) * perceptSlot->block_.worldPoint);
        }

        if (trackSlot->relativeSlots.back()->block_.type != FusionSlot::BlockPoint::Type::NO_BLOCK) {
            const auto diffBlock = (trackSlot->relativeSlots.back()->block_.worldPoint - blockWorldPointTmp).norm();
            if (diffBlock < 1.0) {
                trackSlot->block_.worldPoint = trackSlot->relativeSlots.back()->block_.worldPoint;
            }
        }
    }
}

bool PldFusion::isBlockInGoodObservationBlock(const FusionSlot::Ptr& fusionSlot, PldFrame& frame)
{
    fusionSlot->TransferToBase(frame.poseOb);
    
    Eigen::Vector3d centerOpenPoint(0.0, 0.0, 0.0);
    for (uint16_t i = 0; i < 2U; ++i) {
        centerOpenPoint += fusionSlot->corners[i].baseCorner;
    }
    centerOpenPoint /= 2.0;
    
    if(centerOpenPoint.norm() < 2.5f)
    {
        return true;    
    }
    
    return false;
}


void PldFusion::GenerateNewTrackSlots(
    const std::vector<FusionSlot::Ptr>& noMatchSlots, 
    CSlotsList& trackSlots)
{
    for (const auto& dSlot : noMatchSlots) {
        if(dSlot->getIsLengthTooShort() == true) {
            continue;
        }
        trackSlots.add(dSlot);
    }
}

void PldFusion::Output(PldFrame& frame)
{
    std::list<FusionSlot::Ptr> newOutputSlots_;
    newOutputSlots_.clear();


    for (auto& tSlot : trackSlots_.slotsList_) {
        //bool isCovValid = std::sqrt(tSlot->corners[0].covError.cov) < 0.5 && std::sqrt(tSlot->corners[1].covError.cov) < 0.5;
        if ((tSlot->lifeCounter > 5 || tSlot->isConfirm) /*&& isCovValid*/) {
            tSlot->TransferToBase(frame.poseOb);
            
            tSlot->width = (tSlot->corners[0].worldCorner - tSlot->corners[1].worldCorner).norm();
            tSlot->length = 0.5 * ((tSlot->corners[1].worldCorner - tSlot->corners[2].worldCorner).norm() + (tSlot->corners[3].worldCorner - tSlot->corners[0].worldCorner).norm());

            slotTypeHandle(tSlot);
            // if( (tSlot->type == FusionSlot::Type::INCLINED) 
            //  || (tSlot->type == FusionSlot::Type::VERTICAL)) {
            //     VertcialInclineDepthHandle(tSlot);
            // }

            if((tSlot->id == debugParameter.debugSlotId)||(tSlot->id == debugParameter.debugSlotId2)) {
                //std::cout<<"slot id: "<<tSlot->id<<" occupied state: ";
                if(tSlot->finalStatus == FusionSlot::Status::OCCUPIED){
                    //std::cout<<"occupied"<<"\n";
                }
                else {
                    //std::cout<<"not occupied"<<"\n";
                }
            }

            addOutputSlot(tSlot, outputSlots_, newOutputSlots_);
        }
    }

    for(auto iter = outputSlots_.begin(); iter !=outputSlots_.end();) {
        bool isFound(false);
        for(auto& newSlot : newOutputSlots_) {
            if(*iter == newSlot) {
                isFound = true;
                break;
            }
        }
        if(!isFound) {
            (*iter)->setPtrToNullptr();
            iter = outputSlots_.erase(iter);
        }
        else {
            ++iter;
        }
    }

    outputSlots_.clear();
    outputSlots_ = newOutputSlots_;

    for(auto& slot : outputSlots_) {
        slot->initLines();
        slot->TransferToBase(frame.poseOb);
        slot->calcMiddlePoint(frame.poseOb);
    }

    FixOutput(outputSlots_, frame.poseOb);

    for(auto& trackSlot : trackSlots_.slotsList_) {
        for(int i = 0; i < 4; i++) {
            trackSlot->lines_[i]->lastFrameLength = trackSlot->lines_[i]->getLength();
        }

        for(auto& outslot : outputSlots_) {
            if(outslot->id == trackSlot->id) {
                for(int i = 0; i < 4; i++) {
                    trackSlot->lines_[i]->lastFrameLength = outslot->lines_[i]->getLength();
                }
                break;
            }
        }
    }

    #if DEBUG_MODE
    bool isInParkIn = isParking_;
    if(isInParkIn && (!isLastFrameParkingIn_)) {
        for(auto& mSlot: mergedSlots_.slotsList_) {
            const auto newSlot = std::make_shared<FusionSlot>(*mSlot);
            slotsIntoParkInMode_.slotsList_.push_back(newSlot);
        }
    }
    else if((!isInParkIn) && isLastFrameParkingIn_) {
        slotsIntoParkInMode_.reset();
    }
    else {
        //do nothing
    }
    #endif

    isLastFrameParkingIn_ = isInParkIn;

    //std::cout<<"before merge"<<"\n";
    PldFusion::slotMergeProcess(
        outputSlots_,
        ussSlots_.slotsList_,
        mergedSlots_,
        frame.poseOb);
    //std::cout<<"after merge"<<"\n";
    
    for(auto& slot : mergedSlots_.slotsList_) {
        if((slot->id == debugParameter.debugSlotId)||(slot->id == debugParameter.debugSlotId2)) {
            std::cout<<"slot id: "<<slot->id<<" type: "<<(int)slot->type<<"\n";
        }
    }

    for(auto& mSlot: mergedSlots_.slotsList_) {
        if(mSlot->type == FusionSlot::Type::INCLINED) {
            std::cout<<"inclined slot exit"<<"\n";
        }
    }

    trackSlots_.resetEachCircleForPLDTracking();
}

void PldFusion::slotTypeHandle(const FusionSlot::Ptr tSlot)
{
    if(!isParking_) {
        if(tSlot->type == FusionSlot::Type::INCLINED) {
            if(tSlot->width > tSlot->length) {
                tSlot->finalStatus = FusionSlot::Status::OCCUPIED;
            }

            if(tSlot->block_.type != FusionSlot::BlockPoint::Type::NO_BLOCK) {
                int deepestLineIndex = tSlot->findBlockDeepestLineIndex();
                if(deepestLineIndex != 0) {
                    tSlot->finalStatus = FusionSlot::Status::OCCUPIED;
                }
            }
        }
        else if(tSlot->block_.type != FusionSlot::BlockPoint::Type::NO_BLOCK) {
            int deepestLineIndex = tSlot->findBlockDeepestLineIndex();

            if(deepestLineIndex != 0) {
                tSlot->type = FusionSlot::Type::HORIZONTAL;

                int occupiedIndex = 0;
                if(tSlot->middlePoint.baseCorner[1] > 0.0f) {
                    occupiedIndex = 3;  //for right slot
                }
                else {
                    occupiedIndex = 1;  //for left slot
                }
                if(deepestLineIndex == occupiedIndex) {
                    tSlot->finalStatus = FusionSlot::Status::OCCUPIED;
                }
            }
            else {
                tSlot->type = FusionSlot::Type::VERTICAL;
            }
        }
        else {
            if( (tSlot->width > 4.6)
            && (tSlot->width > tSlot->length ) ) {
                tSlot->type = FusionSlot::Type::HORIZONTAL;
            }
            else {
                if((tSlot->width > tSlot->length)
                && (tSlot->lines_[2]->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::occupiedByNeighborSlot)
                && (tSlot->width > 4.4) ) {
                    tSlot->type = FusionSlot::Type::HORIZONTAL;
                }
                else {
                    //tSlot->length = std::max(tSlot->width * 2.0, tSlot->length);
                    tSlot->length = std::max(1.0, tSlot->length);
                    tSlot->type = FusionSlot::Type::VERTICAL;
                }
            }
            // if((tSlot->id == debugParameter.debugSlotId)||(tSlot->id == debugParameter.debugSlotId2)) {
            //     std::cout<<"slot id: "<<tSlot->id<<" type: "<<(int)tSlot->type<<" width & length: "<<tSlot->width<<" ,"<<tSlot->length<<"\n";
            // }
        }

        if(tSlot->type == FusionSlot::Type::HORIZONTAL) {
            if( (tSlot->width < 4.1) || (tSlot->length < 1.5)) {
                tSlot->finalStatus = FusionSlot::Status::OCCUPIED;

                if(tSlot->id == debugParameter.targetSlotId) {
                    std::cout<<"width & depth not ok"<<"\n";
                }
            }
        }
        else {
            if( (tSlot->width < 1.7) || (tSlot->length < 2.5)) {
                tSlot->finalStatus = FusionSlot::Status::OCCUPIED;
            }
        }
    }
    else {
        if(FusionSlot::Type::VERTICAL == tSlot->type) {
            //tSlot->length = std::max(tSlot->width * 2.0, tSlot->length);
            tSlot->length = std::max(1.0, tSlot->length);
        }
    }
}

void PldFusion::addOutputSlot(
    const FusionSlot::Ptr& slot,
    const std::list<FusionSlot::Ptr> oldOutputSlots,
    std::list<FusionSlot::Ptr>& newOutputSlots)
{ 
    std::array<float, 4U> lastFrameLength;
    for(int i = 0; i < 4; i++) {
        lastFrameLength[i] = slot->lines_[i]->lastFrameLength;
    }

    for (auto& tSlot : oldOutputSlots) {
        if(slot == tSlot->sourceSlot) {
            const auto newSlotOut = std::make_shared<FusionSlot>(*slot);
            tSlot->replace(newSlotOut);
            newOutputSlots.push_back(tSlot);
            tSlot->initLines(lastFrameLength);
            return;
        }
    }

    const auto slotOut = std::make_shared<FusionSlot>(*slot);
    slotOut->sourceSlot = slot;
    newOutputSlots.push_back(slotOut);
    slotOut->initLines(lastFrameLength);
}

void PldFusion::isCornerAsTrunctionPoint(
    const FusionSlot::Ptr imageRange,
    FusionSlot::SlotCorner& corner) 
{
    if(Fusion::isPointInsideSlot(imageRange->lines_, corner.baseCorner) == true) {
        float minDistanceToLine = 2.0f;
        for(int i = 0; i < 4; i++) {
            float distanceToLine = imageRange->lines_[i]->calcOrthDistOfWorldPointAbs(corner.baseCorner);
            if(distanceToLine < minDistanceToLine) {
                minDistanceToLine = distanceToLine;
            }
        }

        //std::cout<<" "<<minDistanceToLine;
        if(minDistanceToLine > 0.3f) {
            corner.isCornerAsTruePoint = true;
            return;
        }
    }

    corner.isCornerAsTruePoint = false;
}

void PldFusion::FixOutput(std::list<FusionSlot::Ptr>& slots, const Odometry& odom)
{
    for (auto& slot : slots) {
        if(slot->type == FusionSlot::Type::HORIZONTAL) {
            ParallelDepthHandle(slot);
        }
        // else {
        //     VertcialInclineDepthHandle(slot);
        // }

        CalculateSlotAngle(slot);
        CalculateBlockDepth(slot);
    }
    
    AlignAjoinedSlots(slots);
    for (auto& slot : slots) {
        if(isParking_ == true) {
            PldFusion::truePointHandle(slot);
        }
        slot->TransferToBase(odom);
        slot->initLines();
        if(slot->id == 1) {
            for(int i = 0; i < 4; i++) {
                std::cout<<i<<" length after handle: "<<slot->lines_[i]->getLength()<<"\n";
            }
        }
        CalculateOtherCorners(slot);
    }

    
}

void PldFusion::ParallelDepthHandle(const FusionSlot::Ptr slot)
{
    float lengthRadio = (float)slot->lines_[0]->getLength()/slot->lines_[2]->getLength();
    //std::cout<<"slot id: "<<slot->id<<" lengthRadio: "<<lengthRadio<<"\n";
    float expectLineLength1 = lengthRadio*lengthRadio*slot->lines_[1]->getLength();
    float expectLineLength3 = lengthRadio*lengthRadio*slot->lines_[3]->getLength();

    slot->corners[2].worldCorner = slot->corners[1].worldCorner + slot->lines_[1]->getDefaultUnitVec()*expectLineLength1;
    slot->corners[3].worldCorner = slot->corners[0].worldCorner + slot->lines_[1]->getDefaultUnitVec()*expectLineLength3;
    slot->length = 0.5 * ((slot->corners[1].worldCorner - slot->corners[2].worldCorner).norm() 
                        + (slot->corners[3].worldCorner - slot->corners[0].worldCorner).norm());

    slot->initLines();
}

void PldFusion::CalculateSlotAngle(FusionSlot::Ptr& fusionSlot)
{
    const Eigen::Vector3d v01 = fusionSlot->corners[1].worldCorner - fusionSlot->corners[0].worldCorner;
    const Eigen::Vector3d v12 = fusionSlot->corners[2].worldCorner - fusionSlot->corners[1].worldCorner;
    const double cosValue1 = v12.dot(v01) / (v01.norm() * v12.norm());

    const Eigen::Vector3d v03 = fusionSlot->corners[3].worldCorner - fusionSlot->corners[0].worldCorner;
    const double cosValue2 = v03.dot(v01) / (v01.norm() * v03.norm());

    fusionSlot->angle = 0.5 * std::acos(cosValue1) + 0.5 * std::acos(cosValue2);
    fusionSlot->angle = (fusionSlot->type == FusionSlot::Type::HORIZONTAL) ? (M_PI / 2.0) : fusionSlot->angle;
}

void PldFusion::VertcialInclineDepthHandle(const FusionSlot::Ptr slot)
{
    if( (1.5f*slot->lines_[0]->getLength() > slot->lines_[1]->getLength())
     || (1.5f*slot->lines_[0]->getLength() > slot->lines_[3]->getLength())) {
        float expectLineLength = 1.5f*slot->lines_[0]->getLength();

        slot->corners[2].worldCorner = slot->corners[1].worldCorner + slot->lines_[1]->getDefaultUnitVec()*expectLineLength;
        slot->corners[3].worldCorner = slot->corners[0].worldCorner - slot->lines_[3]->getDefaultUnitVec()*expectLineLength;
        
        slot->length = 0.5 * ((slot->corners[1].worldCorner - slot->corners[2].worldCorner).norm() 
                        + (slot->corners[3].worldCorner - slot->corners[0].worldCorner).norm());
        slot->initLines();
    }    
}

void PldFusion::CalculateBlockDepth(FusionSlot::Ptr& fusionSlot)
{
    if (fusionSlot->block_.type == FusionSlot::BlockPoint::Type::NO_BLOCK) {
        return;
    }
    
    if(fusionSlot->type == FusionSlot::Type::HORIZONTAL) {
        fusionSlot->block_.depth = fusionSlot->lines_[3]->calcOrthDistOfWorldPointAbs(fusionSlot->block_.worldPoint);
    }
    else {
        // int deepestLineIndex = 0;
        // if(fusionSlot->type == FusionSlot::Type::INCLINED) {
        //     deepestLineIndex = fusionSlot->findBlockDeepestLineIndex();   
        //     if(deepestLineIndex != 0) {
                
        //         fusionSlot->length = std::max(fusionSlot->length, (fusionSlot->block_.depth + 0.7));
        //     }
        // }
        // if(deepestLineIndex != 0) {
        //     return;
        // }
        
        Eigen::Quaterniond quat_sb;
        Eigen::Vector3d p_sb;
        GetSlotCoordTransform(fusionSlot, quat_sb, p_sb);
        Eigen::Vector3d blockPointInSlot = quat_sb * fusionSlot->block_.basePoint + p_sb;
        fusionSlot->block_.depth = std::abs(blockPointInSlot.y() / std::sin(fusionSlot->angle));

        fusionSlot->length = std::max(fusionSlot->length, (fusionSlot->block_.depth + 0.7));
    }
}

void PldFusion::AdjustSlotBlock(FusionSlot::Ptr& fusionSlot)
{
    if (fusionSlot->block_.type == FusionSlot::BlockPoint::Type::NO_BLOCK) {
        fusionSlot->block_.type = FusionSlot::BlockPoint::Type::NO_BLOCK;
        fusionSlot->block_.worldPoint << 0.0, 0.0, 0.0;
        return;
    }
    
    if(fusionSlot->type == FusionSlot::Type::HORIZONTAL) {
        const Eigen::Vector3d midPoint = fusionSlot->corners[0].worldCorner +
            (fusionSlot->corners[3].worldCorner - fusionSlot->corners[0].worldCorner) / 2.0;
        const Eigen::Vector3d depthVec = (fusionSlot->corners[1].worldCorner - fusionSlot->corners[0].worldCorner).normalized();
        fusionSlot->block_.worldPoint = midPoint + (fusionSlot->block_.depth * depthVec); // to do: set parameter
    }
    else {
        const Eigen::Vector3d midPoint = fusionSlot->corners[0].worldCorner +
            (fusionSlot->corners[1].worldCorner - fusionSlot->corners[0].worldCorner) / 2.0;
        const Eigen::Vector3d depthVec = (fusionSlot->corners[3].worldCorner - fusionSlot->corners[0].worldCorner).normalized();
        fusionSlot->block_.worldPoint = midPoint + (fusionSlot->block_.depth * depthVec); // to do: set parameter
    }
}

void PldFusion::GetSlotCoordTransform(const FusionSlot::Ptr& fusionSlot, Eigen::Quaterniond& quat_sb, Eigen::Vector3d& p_sb)
{
    const Eigen::Vector3d v01 = fusionSlot->corners[0].baseCorner - fusionSlot->corners[1].baseCorner;
    const auto heading = std::atan2(v01.y(), v01.x());
    const auto quat_bs = Eular2Quaternion(Eigen::Vector3d(0.0, 0.0, heading));
    quat_sb = quat_bs.inverse();
    p_sb = -(quat_sb * fusionSlot->corners[0].baseCorner);
}

void PldFusion::CalculateOtherCorners(FusionSlot::Ptr& fusionSlot)
{
    const double angular = fusionSlot->angle;
    Eigen::Matrix2d rot;
    rot << cos(angular), -sin(angular), sin(angular), cos(angular);
    Eigen::Vector2d vec01 = (fusionSlot->corners[1].worldCorner - fusionSlot->corners[0].worldCorner).segment<2>(0);

    fusionSlot->corners[2].worldCorner.segment<2>(0) = fusionSlot->corners[1].worldCorner.segment<2>(0) + rot * vec01.normalized() * fusionSlot->length;
    fusionSlot->corners[3].worldCorner.segment<2>(0) = fusionSlot->corners[0].worldCorner.segment<2>(0) + rot * vec01.normalized() * fusionSlot->length;

    AdjustSlotBlock(fusionSlot);
}


void PldFusion::truePointHandle(const FusionSlot::Ptr& fusionSlot)
{
    if( (fusionSlot->type != FusionSlot::Type::VERTICAL) 
     && (fusionSlot->type != FusionSlot::Type::INCLINED) ) {
        return;
    }

    if(fusionSlot->id == 1) {
        for(int i = 0; i < 4; i++) {
            std::cout<<i<<" current length: "<<fusionSlot->lines_[i]->getLength()
                     <<" last frame length: "<<fusionSlot->lines_[i]->lastFrameLength;

            float minDistanceToLine = 1.0f;
            for(int j = 0; j < 4; j++) {
                float distanceToLine = imageRange->lines_[j]->calcOrthDistOfWorldPointAbs(fusionSlot->corners[i].worldCorner);
                if(distanceToLine < minDistanceToLine) {
                    minDistanceToLine = distanceToLine;
                }
            }

            std::cout<<" min dist: "<<minDistanceToLine<<"\n";
        }
    }


    if( (fusionSlot->corners[0].isCornerAsTruePoint == true) 
        && (fusionSlot->corners[1].isCornerAsTruePoint == true)) {
        if( ((fusionSlot->corners[2].isCornerAsTruePoint == false)
            && (fusionSlot->lines_[1]->getLength() < fusionSlot->lines_[1]->lastFrameLength)) 
            || ((fusionSlot->corners[2].isCornerAsTruePoint == true) 
            && (fusionSlot->lines_[1]->getLength() < fusionSlot->lines_[1]->lastFrameLength)
            && ((fusionSlot->lines_[1]->lastFrameLength - fusionSlot->lines_[1]->getLength()) > 1.5f)) ) {
            //reset fusionSlot->corners[2]
            float newLength = fusionSlot->lines_[1]->lastFrameLength;
            Eigen::Vector3d unitVector = fusionSlot->lines_[1]->getDefaultUnitVec();
            fusionSlot->corners[2].worldCorner = fusionSlot->corners[1].worldCorner + unitVector*newLength;
            std::cout<<"corner: "<<2<<" reset"<<"\n";
        }

        if( ((fusionSlot->corners[3].isCornerAsTruePoint == false)
            && (fusionSlot->lines_[3]->getLength() < fusionSlot->lines_[3]->lastFrameLength)) 
            || ((fusionSlot->corners[3].isCornerAsTruePoint == true) 
            && (fusionSlot->lines_[3]->getLength() < fusionSlot->lines_[3]->lastFrameLength)
            && ((fusionSlot->lines_[3]->lastFrameLength - fusionSlot->lines_[3]->getLength()) > 1.5f))) {
            //reset fusionSlot->corners[3]
            float newLength = fusionSlot->lines_[3]->lastFrameLength;
            Eigen::Vector3d unitVector = -fusionSlot->lines_[3]->getDefaultUnitVec();
            fusionSlot->corners[3].worldCorner = fusionSlot->corners[0].worldCorner + unitVector*newLength;
        }
    }

    fusionSlot->length = 0.5 * ((fusionSlot->corners[1].worldCorner - fusionSlot->corners[2].worldCorner).norm() 
                              + (fusionSlot->corners[3].worldCorner - fusionSlot->corners[0].worldCorner).norm());
}


void PldFusion::AlignAjoinedSlots(std::list<FusionSlot::Ptr>& slots)
{
    for(auto iteri = slots.begin(); iteri !=slots.end(); iteri++) {
        for(auto iterj = iteri; iterj != slots.end(); iterj++) {
            if((*iteri) == *(iterj))
            {
                continue;
            }
            const auto v10_i = ((*iteri)->corners[0].worldCorner - (*iteri)->corners[1].worldCorner).normalized();
            const auto v10_j = ((*iterj)->corners[0].worldCorner - (*iterj)->corners[1].worldCorner).normalized();
            const auto dAngle = std::acos(v10_i.dot(v10_j));
            const bool isLikely = (std::abs((*iteri)->width - (*iterj)->width) < 0.5) &&    // to do: set parameter
                (std::abs(dAngle) < (10.0 / 180.0 * M_PI));
            if (isLikely) {
                if (((*iteri)->corners[0].worldCorner - (*iterj)->corners[1].worldCorner).norm() < 0.5) {
                    (*iteri)->ajoin.rightSlot = (*iterj);
                    (*iterj)->ajoin.leftSlot = (*iteri);
                }
                if (((*iteri)->corners[1].worldCorner - (*iterj)->corners[0].worldCorner).norm() < 0.5) {
                    (*iteri)->ajoin.leftSlot = (*iterj);
                    (*iterj)->ajoin.rightSlot = (*iteri);
                }
            }
        }
    }

    // for (size_t i = 0U; i < slots.size(); ++i) {
    //     for (size_t j = i + 1; j < slots.size(); ++j) {
    //         const auto v10_i = (slots[i]->corners[0].worldCorner - slots[i]->corners[1].worldCorner).normalized();
    //         const auto v10_j = (slots[j]->corners[0].worldCorner - slots[j]->corners[1].worldCorner).normalized();
    //         const auto dAngle = std::acos(v10_i.dot(v10_j));
    //         const bool isLikely = (std::abs(slots[i]->width - slots[j]->width) < 0.5) &&    // to do: set parameter
    //             (std::abs(dAngle) < (10.0 / 180.0 * M_PI));
    //         if (isLikely) {
    //             if ((slots[i]->corners[0].worldCorner - slots[j]->corners[1].worldCorner).norm() < 0.5) {
    //                 slots[i]->ajoin.rightSlot = slots[j];
    //                 slots[j]->ajoin.leftSlot = slots[i];
    //             }
    //             if ((slots[i]->corners[1].worldCorner - slots[j]->corners[0].worldCorner).norm() < 0.5) {
    //                 slots[i]->ajoin.leftSlot = slots[j];
    //                 slots[j]->ajoin.rightSlot = slots[i];
    //             }
    //         }
    //     }
    // }

    std::vector<FusionSlot::Ptr> headerSlots;
    for (auto& slot : slots) {
        if ((slot->ajoin.rightSlot != nullptr && slot->ajoin.leftSlot == nullptr)
         || (slot->ajoin.rightSlot == nullptr && slot->ajoin.leftSlot == nullptr) ) {
            headerSlots.push_back(slot);
        }
    }
    for (auto& slot : headerSlots) {
        std::vector<FusionSlot::Ptr> ajoinSlots;
        FindAjoinRightSlot(slot, ajoinSlots);

        if (!ajoinSlots.empty()) {
            MakeAlign(ajoinSlots);
        }
    }
}

void PldFusion::FindAjoinRightSlot(const FusionSlot::Ptr& slot, std::vector<FusionSlot::Ptr>& ajoinSlots)
{
    ajoinSlots.push_back(slot);  
    if (slot->ajoin.rightSlot != nullptr) {
        FindAjoinRightSlot(slot->ajoin.rightSlot, ajoinSlots);    
    }
    slot->ajoin.rightSlot = nullptr;
    slot->ajoin.leftSlot = nullptr;
}

void PldFusion::MakeAlign(const std::vector<FusionSlot::Ptr>& ajoinSlots)
{
    double blockDepthSum = 0.0;
    int16_t blockNum = 0;
    double lengthSum = 0.0;
    double angleSum = 0.0;
    Eigen::Vector3d headingVecSum(0.0, 0.0, 0.0);
    for (auto& slot : ajoinSlots) {
        if (slot->block_.type != FusionSlot::BlockPoint::Type::NO_BLOCK) {
            blockDepthSum += slot->block_.depth;
            ++blockNum;
        }
        lengthSum += slot->length;
        angleSum += slot->angle;
        const Eigen::Vector3d v01 = (slot->corners[0].worldCorner - slot->corners[1].worldCorner).normalized();
        headingVecSum += v01;
    }
    lengthSum /= static_cast<double>(ajoinSlots.size());
    angleSum /= static_cast<double>(ajoinSlots.size());
    if (blockNum != 0) {
        blockDepthSum /= static_cast<double>(blockNum);
        lengthSum = std::max(lengthSum, (blockDepthSum + 0.7));
    }
    headingVecSum /= static_cast<double>(ajoinSlots.size());

    if( (angleSum < (double)(70.0f/180.0f)*M_PI) 
     || (angleSum > (double)(110.0f/180.0f)*M_PI) ) {
        for(auto& slot : ajoinSlots) {
            if(slot->type == FusionSlot::Type::VERTICAL) {
                slot->type = FusionSlot::Type::INCLINED;
            }
        }
     }
}

void PldFusion::FusionSlotEdges(const FusionSlot::Ptr detectSlot, const double fusionScale, FusionSlot::Ptr& trackSlot)
{
    if (detectSlot->edges_[0].valid) {
        if (trackSlot->edges_[0].valid) {
            trackSlot->edges_[0].worldPoint = ((1 - fusionScale) * trackSlot->edges_[0].worldPoint) + (fusionScale * detectSlot->edges_[0].worldPoint);
        } else {
            trackSlot->edges_[0].worldPoint = detectSlot->edges_[0].worldPoint;
        }
        ++trackSlot->edges_[0].lifeCounter;
        trackSlot->edges_[0].valid = detectSlot->edges_[0].valid;
    } else {
        --trackSlot->edges_[0].lifeCounter;
    }
    trackSlot->edges_[0].lifeCounter = std::min(trackSlot->edges_[0].lifeCounter, 20);
    trackSlot->edges_[0].lifeCounter = std::max(trackSlot->edges_[0].lifeCounter, 0);


    if (detectSlot->edges_[1].valid) {
        if (trackSlot->edges_[1].valid) {
            trackSlot->edges_[1].worldPoint = (1 - fusionScale) * trackSlot->edges_[1].worldPoint + (fusionScale * detectSlot->edges_[1].worldPoint);
        } else {
            trackSlot->edges_[1].worldPoint = detectSlot->edges_[1].worldPoint;
        }
        ++trackSlot->edges_[1].lifeCounter;
        trackSlot->edges_[1].valid = detectSlot->edges_[1].valid;
    } else {
        --trackSlot->edges_[1].lifeCounter;
    }
    trackSlot->edges_[1].lifeCounter = std::min(trackSlot->edges_[1].lifeCounter, 20);
    trackSlot->edges_[1].lifeCounter = std::max(trackSlot->edges_[1].lifeCounter, 0);
}

void PldFusion::Publish(PldFrame& frame)
{

    SlotsListMsg pldMsg = PubUssSlot(outputSlots_, frame.poseOb, 0);
    slotsPub_->publish(pldMsg);

    SlotsListMsg MergedMsg = PubUssSlot(mergedSlots_.slotsList_, frame.poseOb, 1);
    MergedSlotsPub_->publish(MergedMsg);

#if DEBUG_MODE
    SlotsListMsg ussMsg = PubUssSlot(ussSlots_.slotsList_, frame.poseOb, 0);
    ussSlotsPub_->publish(ussMsg);
#endif
}

SlotsListMsg PldFusion::PubSlot(
    const std::vector<FusionSlot::Ptr>& slots,
    const Odometry& poseOb)
{
    SlotsListMsg msgOut;
    msgOut.lads_perception_fusion_header.time_stamp = poseOb.timestamp.NSec();
    msgOut.parking_slots_num = std::min(slots.size(), static_cast<size_t>(30U));
    msgOut.status = static_cast<uint8_t>(slots.size() > 30);
    for (size_t i = 0U; (i < 30U && i < slots.size()); ++i) {
        slots[i]->TransferToBase(poseOb);
        msgOut.slot_list[i] = slots[i]->ToMsg(0);
    }
    return msgOut;
}


void PldFusion::resetPldNode()
{
    trackSlots_.reset();
    ussSlots_.reset();
    mergedSlots_.reset();

#if DEBUG_MODE
    slotsIntoParkInMode_.reset();
#endif

    for(auto iter = outputSlots_.begin(); iter !=outputSlots_.end();) {

        (*iter)->setPtrToNullptr();
        iter = outputSlots_.erase(iter);
    }
    outputSlots_.clear();
    isUpdate_ = false;
    isParking_ = false;

}


#if DEBUG_MODE

//mode  = 0, others,
//        1, for fusion slots
SlotsListMsg PldFusion::PubUssSlot(
    const std::list<FusionSlot::Ptr>& ussSlots, 
    const Odometry& poseOb,
    const int mode)
{
    SlotsListMsg ussMsgOut;
    ussMsgOut.lads_perception_fusion_header.time_stamp = poseOb.timestamp.NSec();

    ussMsgOut.parking_slots_num = 0;
    for (auto& tSlot : ussSlots) {
        if(30 > ussMsgOut.parking_slots_num) {
            //tSlot->TransferToBase(poseOb);
            ussMsgOut.slot_list[ussMsgOut.parking_slots_num] = tSlot->ToMsg(mode);
            ussMsgOut.parking_slots_num++;
        }
    }

    return ussMsgOut;
}
#endif

void PldFusion::Debug(PldFrame& frame)
{
#if DEBUG_MODE
    for (const auto& tSlot : outputSlots_) {
        tSlot->TransferToBase(frame.poseOb);
    }
    ShowFusionPld(outputSlots_);

    for (const auto& tSlot : slotsIntoParkInMode_.slotsList_) {

        int targetID(0);
        if( (tSlot->id == debugParameter.targetSlotId)) {
            targetID = tSlot->id;
            
            PldFusion::p0p1ErrorHandle(
                tSlot,
                mergedSlots_.slotsList_,
                targetID);
        }

        tSlot->TransferToBase(frame.poseOb);
    }
    ShowSlotsIntoParkinMode(slotsIntoParkInMode_.slotsList_);
    
#endif
}



#if DEBUG_MODE
void PldFusion::p0p1ErrorHandle(
    const FusionSlot::Ptr tSlot,
    const std::list<FusionSlot::Ptr> mergedSlotsList,
    const int targetID)
{
    tSlot->initLines();
    std::cout<<"slot id: "<<tSlot->id<<"\n";

    Eigen::Vector3d worldP0 = Eigen::Vector3d(tSlot->corners[0].worldCorner[0], tSlot->corners[0].worldCorner[1], 0.0);
    Eigen::Vector3d worldP1 = Eigen::Vector3d(tSlot->corners[1].worldCorner[0], tSlot->corners[1].worldCorner[1], 0.0);
    float tauP0 = tSlot->lines_[0]->calcTauOfOrthProj(worldP0);
    float tauP1 = tSlot->lines_[0]->calcTauOfOrthProj(worldP1);

    for (const auto& outSlot : mergedSlotsList) {
        if(outSlot->id == targetID) {

            std::cout<<"track slot: "<<"\n";
            PldFusion::calcPointsDiffInSlotCoordinate(
                outSlot, 
                tSlot,
                tauP0, 
                tauP1,
                worldP0,
                worldP1);

            if( (outSlot->alignInfo.pldSlot != nullptr) 
            && (outSlot->alignInfo.pldSlot->sourceSlot != nullptr) 
            && (outSlot->alignInfo.pldSlot->sourceSlot->relativeSlots.size() > 0) ) {
                
                auto& perceptSlot = outSlot->alignInfo.pldSlot->sourceSlot->relativeSlots.at(outSlot->alignInfo.pldSlot->sourceSlot->relativeSlots.size() -1);

                //FusionSlot::Ptr lastInputSlot = outSlot->alignInfo.pldSlot->relativeSlots.Back();
                std::cout<<"last input slot: "<<"\n";
                PldFusion::calcPointsDiffInSlotCoordinate(
                    perceptSlot, 
                    tSlot,
                    tauP0, 
                    tauP1,
                    worldP0,
                    worldP1);
            }

            //std::cout<<"dist P0: "<<(worldP0 - outWorldP0).norm()<<"\n";
            //std::cout<<"dist P1: "<<(worldP1 - outWorldP1).norm()<<"\n";
            break;
        }
    }

}


void PldFusion::calcPointsDiffInSlotCoordinate(
    const FusionSlot::Ptr outSlot, 
    const FusionSlot::Ptr tSlot,
    const float tauP0, 
    const float tauP1,
    const Eigen::Vector3d worldP0,
    const Eigen::Vector3d worldP1)
{
    Eigen::Vector3d outWorldP0 = 
        Eigen::Vector3d(outSlot->corners[0].worldCorner[0], outSlot->corners[0].worldCorner[1], 0.0);
    Eigen::Vector3d outWorldP1 = 
        Eigen::Vector3d(outSlot->corners[1].worldCorner[0], outSlot->corners[1].worldCorner[1], 0.0);

    float tauOutP0 = tSlot->lines_[0]->calcTauOfOrthProj(outWorldP0);
    float tauOutP1 = tSlot->lines_[0]->calcTauOfOrthProj(outWorldP1);

    float diffTauP0 = tauOutP0 - tauP0;
    std::string p0Side;
    if(!tSlot->lines_[0]->getIsUnitVectorInversed()) {
        if(diffTauP0 < 0) {
            p0Side = "right";
        }
        else {
            p0Side = "left";
        }
    }
    else {
        if(diffTauP0 > 0) {
            p0Side = "right";
        }
        else {
            p0Side = "left";
        }      
    }

    float diffTauP1 = tauOutP1 - tauP1;
    std::string p1Side;
    if(!tSlot->lines_[0]->getIsUnitVectorInversed()) {
        if(diffTauP1 < 0) {
            p1Side = "right";
        }
        else {
            p1Side = "left";
        }
    }
    else {
        if(diffTauP1 > 0) {
            p1Side = "right";
        }
        else {
            p1Side = "left";
        }      
    }

    std::cout<<"dist P0: "<<std::fabs(diffTauP0)<<" "<<p0Side<<"\n";
    std::cout<<"dist P1: "<<std::fabs(diffTauP1)<<" "<<p1Side<<"\n";
    std::cout<<"p0: "<<outWorldP0[0]<<", "<<outWorldP0[1]<<"\n";
    std::cout<<"p1: "<<outWorldP1[0]<<", "<<outWorldP1[1]<<"\n";


    Fusion::CTrackLine::Ptr verticalLine = std::make_shared<Fusion::CTrackLine>();
    tSlot->lines_[0]->getVerticalLine(verticalLine);

    float verticalTauP0 = verticalLine->calcTauOfOrthProj(verticalLine->getStartPoint());
    float verticalTauP1 = verticalTauP0; //verticalLine->calcTauOfOrthProj(verticalLine->getEndPoint());
    
    float verticalTauOutP0 = verticalLine->calcTauOfOrthProj(outWorldP0);
    float verticalTauOutP1 = verticalLine->calcTauOfOrthProj(outWorldP1);

    float diffVerticalTauP0 = verticalTauOutP0 - verticalTauP0;
    std::string verticalP0Side;
    if(!verticalLine->getIsUnitVectorInversed()) {
        if(diffVerticalTauP0 < 0) {
            verticalP0Side = "vor";
        }
        else {
            verticalP0Side = "nach";
        }
    }
    else {
        if(diffVerticalTauP0 > 0) {
            verticalP0Side = "vor";
        }
        else {
            verticalP0Side = "nach";
        }      
    }

    float diffVerticalTauP1 = verticalTauOutP1 - verticalTauP1;
    std::string verticalP1Side;
    if(!verticalLine->getIsUnitVectorInversed()) {
        if(diffVerticalTauP1 < 0) {
            verticalP1Side = "vor";
        }
        else {
            verticalP1Side = "nach";
        }
    }
    else {
        if(diffVerticalTauP1 > 0) {
            verticalP1Side = "vor";
        }
        else {
            verticalP1Side = "nach";
        } 
    }

    // std::cout<<"dist P0: "<<std::fabs(diffTauP0)<<" "<<p0Side<<" vertical: "<<std::fabs(diffVerticalTauP0)<<" "<<verticalP0Side<<"\n";
    // std::cout<<"dist P1: "<<std::fabs(diffTauP1)<<" "<<p1Side<<" vertical: "<<std::fabs(diffVerticalTauP1)<<" "<<verticalP1Side<<"\n";
    // std::cout<<"p0: "<<outWorldP0[0]<<", "<<outWorldP0[1]<<"\n";
    // std::cout<<"p1: "<<outWorldP1[0]<<", "<<outWorldP1[1]<<"\n";
    
    


}

void PldFusion::ShowFusionPld(const std::list<FusionSlot::Ptr>& slots)
{
    visualization_msgs::msg::MarkerArray markers;
    for (auto& tSlot : slots) {
        PushSingleFusionPldMarker(tSlot, markers, 0);
        PushSlotId(tSlot, markers);
        PushSlotBlock(tSlot, markers, 0);
        PushSlotDirection(tSlot, markers);
        // PushEdges(slots[i], i, markers);
    }
    fusionSlotMarkerPub_->publish(markers);
}

void PldFusion::ShowSlotsIntoParkinMode(const std::list<FusionSlot::Ptr>& slots)
{
    visualization_msgs::msg::MarkerArray markers;
    for (auto& tSlot : slots) {
        PushSingleFusionPldMarker(tSlot, markers, 1);
        PushSlotBlock(tSlot, markers, 1);
    }
    slotsTheMomentIntoParkInMode_pub->publish(markers);
}

void PldFusion::PushSingleFusionPldMarker(
    const FusionSlot::Ptr& fusionSlot, 
    visualization_msgs::msg::MarkerArray& markers,
    const int mode)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = nh_.get_clock()->now();
    marker.ns = "fusion_slot";
    marker.id = fusionSlot->id;
    marker.type = 4;
    marker.action = 0;

    marker.scale.x = debugParameter.showMarkerScale;
    marker.scale.y = debugParameter.showMarkerScale;
    marker.scale.z = debugParameter.showMarkerScale;

    if(mode == 0) {
        std_msgs::msg::ColorRGBA color;
        color.r = 0;
        color.g = 1;
        color.b = 0;
        color.a = 1;
        marker.color = color;

        if(debugParameter.showOpenLine == false) {
            if (fusionSlot->finalStatus == FusionSlot::Status::OCCUPIED) {
                SetOccupiedMarker(fusionSlot, marker);
            } else {
                SetAvailableMarker(fusionSlot, marker);
            }
        }
        else {
            SetAvailableMarker(fusionSlot, marker);
        }
    }
    else if(mode == 1) {
        std_msgs::msg::ColorRGBA color;
        color.r = 1;
        color.g = 0;
        color.b = 0;
        color.a = 1;
        marker.color = color;  
        SetAvailableMarker(fusionSlot, marker); 
    }

    markers.markers.emplace_back(marker);
}

void PldFusion::SetAvailableMarker(const FusionSlot::Ptr& fusionSlot, visualization_msgs::msg::Marker& marker)
{
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    for (uint32_t i = 1U; i < 5; ++i) {
        geometry_msgs::msg::Point point;
        const auto index = i % 4U;
        point.x = fusionSlot->corners[index].baseCorner.x();
        point.y = fusionSlot->corners[index].baseCorner.y();
        point.z = 0.5;
        marker.points.push_back(std::move(point));
        marker.colors.push_back(marker.color);
    }
}

void PldFusion::SetOccupiedMarker(const FusionSlot::Ptr& fusionSlot, visualization_msgs::msg::Marker& marker)
{
    std_msgs::msg::ColorRGBA color;
    color.r = 1;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    if (fusionSlot->isOccupiedByVeh) {
        color.g = 0.5;
    }
    else if(fusionSlot->isOccupiedByLocker) {
        color.g = 1.0;
    }
    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    for (uint32_t i = 0U; i < 5; ++i) {
        geometry_msgs::msg::Point point;
        const auto index = i % 4U;
        point.x = fusionSlot->corners[index].baseCorner.x();
        point.y = fusionSlot->corners[index].baseCorner.y();
        point.z = 0.5;
        marker.points.push_back(std::move(point));
        marker.colors.push_back(color);
    }
    {
        const int32_t index = 2;
        geometry_msgs::msg::Point point;
        point.x = fusionSlot->corners[index].baseCorner.x();
        point.y = fusionSlot->corners[index].baseCorner.y();
        point.z = 0.5;
        marker.points.push_back(std::move(point));
        marker.colors.push_back(color);
    }
    
    {
        const int32_t index = 3;
        geometry_msgs::msg::Point point;
        point.x = fusionSlot->corners[index].baseCorner.x();
        point.y = fusionSlot->corners[index].baseCorner.y();
        point.z = 0.5;
        marker.points.push_back(std::move(point));
        marker.colors.push_back(color);
    }
    
    {
        const int32_t index = 1;
        geometry_msgs::msg::Point point;
        point.x = fusionSlot->corners[index].baseCorner.x();
        point.y = fusionSlot->corners[index].baseCorner.y();
        point.z = 0.5;
        marker.points.push_back(std::move(point));
        marker.colors.push_back(color);
    } 
}


// void PldFusion::PushCornerIndex(const FusionSlot::Ptr& fusionSlot, visualization_msgs::msg::MarkerArray& markers)
// {
//     visualization_msgs::msg::Marker marker;
//     marker.header.frame_id = "base_link";
//     marker.header.stamp = nh_.get_clock()->now();
//     marker.lifetime = rclcpp::Duration::from_seconds(0.2);
//     marker.ns = "id";
//     marker.id = fusionSlot->id;
//     marker.type = 9;
//     marker.action = 0;

//     marker.color.r = 0;
//     marker.color.g = 1;
//     marker.color.b = 0;
//     marker.color.a = 1;

//     marker.scale.x = 0.4;
//     marker.scale.y = 0.4;
//     marker.scale.z = 0.4;
//     for(int i = 0; i < 4; i++) {
//         marker.pose.position.x = fusionSlot->corners[i].baseCorner.x();
//         marker.pose.position.y = fusionSlot->corners[i].baseCorner.y();
//         marker.pose.position.z = fusionSlot->corners[i].baseCorner.z();
//         marker.pose.orientation.w = -1.0;
//         marker.pose.orientation.x = 0.0;
//         marker.pose.orientation.y = 0.0;
//         marker.pose.orientation.z = 0.0;
//         marker.text = std::to_string(i);
//     }
// }


void PldFusion::PushSlotId(const FusionSlot::Ptr& fusionSlot, visualization_msgs::msg::MarkerArray& markers)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = nh_.get_clock()->now();
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.ns = "id";
    marker.id = fusionSlot->id;
    marker.type = 9;
    marker.action = 0;

    marker.scale.x = 0.8;
    marker.scale.y = 0.8;
    marker.scale.z = 0.8;

    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;

    Eigen::Vector3d centerPoint(0.0, 0.0, 0.0);
    for (uint16_t i = 0; i < 4U; ++i) {
        centerPoint += fusionSlot->corners[i].baseCorner;
    }
    centerPoint /= 4.0;
    marker.pose.position.x = centerPoint.x();
    marker.pose.position.y = centerPoint.y();
    marker.pose.position.z = centerPoint.z();

    marker.pose.orientation.w = -1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;

    marker.text = std::to_string(fusionSlot->id);

    std::string type;
    if(fusionSlot->type == FusionSlot::Type::HORIZONTAL) {
        type = " parallel";
    }
    else if(fusionSlot->type == FusionSlot::Type::VERTICAL) {
        type = " vertical";
    }
    else if(fusionSlot->type == FusionSlot::Type::INCLINED) {
        type = " inclined";
    }
    marker.text += type;

    markers.markers.emplace_back(marker);
}

void PldFusion::PushSlotBlock(const FusionSlot::Ptr& fusionSlot, visualization_msgs::msg::MarkerArray& markers, const int mode)
{
    if (fusionSlot->block_.type == FusionSlot::BlockPoint::Type::NO_BLOCK) {
        return;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = nh_.get_clock()->now();
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.ns = "block";
    marker.id = fusionSlot->id;
    marker.type = 4;
    marker.action = 0;
    if(mode == 0){
        marker.scale.x = 0.1;
        marker.scale.y = 0.4;
        marker.scale.z = 0.1;

        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0;
        marker.color.a = 1;
    }
    else if(mode == 1) {
        marker.scale.x = 0.1;
        marker.scale.y = 0.4;
        marker.scale.z = 0.1;

        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 1; 
    }

    Eigen::Vector3d v23;
    if(fusionSlot->type == FusionSlot::Type::HORIZONTAL) {
        v23 = (fusionSlot->corners[2].baseCorner - fusionSlot->corners[1].baseCorner).normalized();
    }
    else {
        v23 = (fusionSlot->corners[3].baseCorner - fusionSlot->corners[2].baseCorner).normalized();
    }

    {
        Eigen::Vector3d blockPointStart = fusionSlot->block_.basePoint + v23 * 0.4;
        geometry_msgs::msg::Point point;
        point.x = blockPointStart.x();
        point.y = blockPointStart.y();
        point.z = 0.5;
        marker.points.push_back(std::move(point));
        marker.colors.push_back(marker.color);
    }
    
    {
        Eigen::Vector3d blockPointEnd = fusionSlot->block_.basePoint - v23 * 0.4;
        geometry_msgs::msg::Point point;
        point.x = blockPointEnd.x();
        point.y = blockPointEnd.y();
        point.z = 0.5;
        marker.points.push_back(std::move(point));
        marker.colors.push_back(marker.color);
    }

    markers.markers.emplace_back(marker);
}

void PldFusion::PushSlotDirection(const FusionSlot::Ptr& fusionSlot, visualization_msgs::msg::MarkerArray& markers)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = nh_.get_clock()->now();
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.ns = "direction";
    marker.id = fusionSlot->id;
    marker.type = 0;
    marker.action = 0;

    marker.scale.x = 0.8;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;

    Eigen::Vector3d centerPoint(0.0, 0.0, 0.0);
    for (uint16_t i = 0; i < 4U; ++i) {
        centerPoint += fusionSlot->corners[i].baseCorner;
    }
    centerPoint /= 4.0;
    marker.pose.position.x = centerPoint.x();
    marker.pose.position.y = centerPoint.y();
    marker.pose.position.z = centerPoint.z();
    const auto v10 = (fusionSlot->corners[0].baseCorner - fusionSlot->corners[1].baseCorner).normalized();
    const double heading = std::atan2(v10.y(), v10.x());
    const Eigen::Quaterniond quat = Eular2Quaternion(Eigen::Vector3d(0.0, 0.0, heading));
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();

    markers.markers.emplace_back(marker);
}

void PldFusion::PushEdges(const FusionSlot::Ptr& fusionSlot, const uint16_t id, visualization_msgs::msg::MarkerArray& markers)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = nh_.get_clock()->now();
    marker.ns = "edges";
    marker.id = id;
    marker.type = 8;
    marker.action = 0;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    std_msgs::msg::ColorRGBA color;
    color.r = 1;
    color.g = 1;
    color.b = 0;
    color.a = 1;
    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    for (uint32_t i = 0U; i < 2; ++i) {
        geometry_msgs::msg::Point point;
        const auto index = i % 2U;
        if (!fusionSlot->edges_[index].valid || fusionSlot->edges_[index].lifeCounter == 0) {
            continue;
        }
        point.x = fusionSlot->edges_[index].point.x();
        point.y = fusionSlot->edges_[index].point.y();
        point.z = 0.5;
        marker.points.push_back(std::move(point));
        marker.colors.push_back(color);
    }

    markers.markers.emplace_back(marker);
}
#endif

}
}
