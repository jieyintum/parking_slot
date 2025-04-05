#include "pld_fusion/pld_fusion.h"

#include "utils/matrix_utils.h"

#include <iostream>

namespace Fusion {
namespace PSE {


void PldFusion::matchAlgo(PldFrame& frame, std::vector<FusionSlot::Ptr>& noMatchSlots)
{
    for (auto& tSlot : trackSlots_.slotsList_) {
        tSlot->initLines();
    }

    for(int i = 0; i < frame.detectSlots_.size(); i++) {
        frame.detectSlots_[i]->initLines();
        
        bool l_isMatched_b = false;
        
        for(auto& tSlot : trackSlots_.slotsList_) {
            l_isMatched_b = PldFusion::getMatchedInfo(tSlot, frame.detectSlots_[i]);
        }

        if(l_isMatched_b == false) {
            if(PldFusion::isConflictWithOtherTrackSlot(trackSlots_.slotsList_, frame.detectSlots_[i]) == false) {
                noMatchSlots.push_back(frame.detectSlots_[i]);
            }
        }
    }
    

    while(false == PldFusion::isAllInfoofMatchedPSBeenProcessedFinished(frame.detectSlots_))
    {
        for(int i = 0; i < frame.detectSlots_.size(); i++) {
            FusionSlot::Ptr& newSlot = frame.detectSlots_[i];
            
            if( (newSlot->matchedList.size() > 0) 
             && ( ((*(newSlot->matchedList.end() - 1)).isDispatched == false)
                 && (newSlot->lowestCostInfo.matchedSlot == nullptr) ) )
            {
                for(int itr = 0; itr < newSlot->matchedList.size(); itr++) {
                    
                    FusionSlot::matchedInfo& tempMatchedInfo = newSlot->matchedList[itr];
                    if(tempMatchedInfo.isDispatched == true) {
                        continue;
                    }
                    tempMatchedInfo.isDispatched = true;
                    
                    FusionSlot::Ptr tempMatchedTrackSlot = tempMatchedInfo.matchedSlot;
                    if(tempMatchedTrackSlot->lowestCostInfo.matchedSlot == nullptr) {
                        tempMatchedTrackSlot->lowestCostInfo = tempMatchedInfo;
                        tempMatchedTrackSlot->lowestCostInfo.matchedSlot = newSlot;

                        newSlot->lowestCostInfo = tempMatchedInfo;
                        break;
                    }
                    else {
                        if(tempMatchedTrackSlot->lowestCostInfo > tempMatchedInfo) {
                            newSlot->isShouldNotInited = true;
                        }
                        else {
                            tempMatchedTrackSlot->lowestCostInfo.matchedSlot->isShouldNotInited = true;
                            tempMatchedTrackSlot->lowestCostInfo.matchedSlot->lowestCostInfo.matchedSlot = nullptr;

                            tempMatchedTrackSlot->lowestCostInfo = tempMatchedInfo;
                            tempMatchedTrackSlot->lowestCostInfo.matchedSlot = newSlot;
                            
                            newSlot->lowestCostInfo = tempMatchedInfo;
                            break;
                        }
                    }
                }
            }
        }
    }


    // if( (slot10 != nullptr)
    //  && (slot12 != nullptr) ) {
    //     std::cout<<"slot10: "<<slot10->lowestCostInfo.matchedSlot<<" is cofirmed: " <<slot10->isConfirm<<" cout: "<<slot10->lifeCounter<<"\n";
    //     std::cout<<"slot12: "<<slot12->lowestCostInfo.matchedSlot<<" is cofirmed: " <<slot12->isConfirm<<" cout: "<<slot12->lifeCounter<<"\n";
    // }
}


bool PldFusion::isAllInfoofMatchedPSBeenProcessedFinished(const std::vector<FusionSlot::Ptr> detectSlots)
{
    for(int itr = 0; itr < detectSlots.size(); itr++) {

        if(detectSlots[itr]->matchedList.size() == 0) {
            //do nothing
        }
        else if( (detectSlots[itr]->lowestCostInfo.matchedSlot == nullptr) 
              && ((*(detectSlots[itr]->matchedList.end() - 1)).isDispatched == false) )
        {
            return false;
        }
    }

    return true;
}


bool PldFusion::isConflictWithOtherTrackSlot(
    const std::list<FusionSlot::Ptr> trackSlotsList,
    const FusionSlot::Ptr slot)
{
    for(const auto& trackSlot_ :  trackSlotsList) {
        if(Fusion::isPointInsideSlot(trackSlot_->lines_, slot->middlePoint.worldCorner)
        || Fusion::isPointInsideSlot(slot->lines_, trackSlot_->middlePoint.worldCorner)) {
            return true;
        }
    }
    return false;
}


bool PldFusion::getMatchedInfo(
    const FusionSlot::Ptr trackSlot,
    const FusionSlot::Ptr newSlot)
{
    bool l_returnValue_b(false);
    int l_numOfMatchedLines_i(0);
    float l_sumOfOrthError_f(0.0f);

    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            float l_errorOfOrthDist_f(0.0f);
            float l_minOverlapRatio_f(0.0f);

            if(true == PldFusion::isLinesMatched(
                trackSlot->lines_[i], 
                newSlot->lines_[j], 
                l_errorOfOrthDist_f, 
                l_minOverlapRatio_f))
            {
                l_returnValue_b = true;
                l_numOfMatchedLines_i++;
                l_sumOfOrthError_f += l_errorOfOrthDist_f;
                break;
            }
        }
    }

    if(0 < l_numOfMatchedLines_i)
    {
        FusionSlot::matchedInfo matchedInfo_;
        
        matchedInfo_.matchedSlot = newSlot;
        matchedInfo_.numOfMatchedLines = l_numOfMatchedLines_i;
        matchedInfo_.distSumOfMatchedLines = l_sumOfOrthError_f;

        trackSlot->addMatchedInfo(matchedInfo_);

        matchedInfo_.matchedSlot = trackSlot;
        newSlot->addMatchedInfo(matchedInfo_);
    }


    return l_returnValue_b;
}





bool PldFusion::checkIfOverlapOK(
    const Fusion::CTrackLine::Ptr line1,
    const Fusion::CTrackLine::Ptr line2,
    float& f_minOverlapRatio_f)
{   
    float l_overlapsLength_f(0.0f);
    
    bool l_isOverlap_b = 
        line1->isOverlap(line2->getStartPoint(), line2->getEndPoint(), l_overlapsLength_f);
    
    if(false == l_isOverlap_b)
    {
        return false;
    }
    else
    {
        float l_overlapRatioA_f = (float)(l_overlapsLength_f/line1->getLength());
        float l_overlapRatioB_f = (float)(l_overlapsLength_f/line2->getLength());
        f_minOverlapRatio_f = std::min(l_overlapRatioA_f, l_overlapRatioB_f);

        if( (0.5 > l_overlapRatioA_f)
         && (0.5 > l_overlapRatioB_f) )
        {
            return false;
        }
        else
        {
            return true;   
        }
    }
}





bool PldFusion::isLinesMatched(
    const Fusion::CTrackLine::Ptr line1,
    const Fusion::CTrackLine::Ptr line2,
    float& f_errorOfOrthDist_f,
    float& f_minOverlapRatio_f)
{
    bool l_isOrthDisOk_b(false);
    bool l_isAngleDifferOK_b(false);
    bool l_isOverlapOK_b(false);

    float l_orthDistOfSP_f = line1->calcOrthDistOfWorldPointAbs(line2->getStartPoint());
    float l_orthDistOfEP_f = line1->calcOrthDistOfWorldPointAbs(line2->getEndPoint());

    if( (0.8f > l_orthDistOfSP_f)
     && (0.8f > l_orthDistOfEP_f) )
    {
        l_isOrthDisOk_b = true;
        f_errorOfOrthDist_f = l_orthDistOfSP_f + l_orthDistOfEP_f;
    }
    else {
        return false;
    }

    float l_deltaAngle_in_Degree_f = Fusion::calcAngleBetweenTwoUnitVectorInDegree(
        line1->getDefaultUnitVec(),
        line2->getDefaultUnitVec());
    
    if(35.0f > std::fabs(l_deltaAngle_in_Degree_f))
    {
        l_isAngleDifferOK_b = true;
    }
    else {
        return false;
    }

    l_isOverlapOK_b = PldFusion::checkIfOverlapOK(line1, line2, f_minOverlapRatio_f);
    return l_isOverlapOK_b;
}




}
}