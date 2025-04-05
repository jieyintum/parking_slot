#include "pld_fusion/pld_fusion.h"

#include "utils/matrix_utils.h"

#include <iostream>

namespace Fusion {
namespace PSE {

void PldFusion::openLineCalc(
    std::list<FusionSlot::Ptr>& trackSlots,
    PldFrame& frame)
{
    for(auto& tSlot : trackSlots) {
        tSlot->type = FusionSlot::Type::UNKNOWN;
        tSlot->initLines();

        const Eigen::Vector3d v01 = tSlot->corners[1].worldCorner - tSlot->corners[0].worldCorner;
        const Eigen::Vector3d v12 = tSlot->corners[2].worldCorner - tSlot->corners[1].worldCorner;
        const double cosValue1 = v12.dot(v01) / (v01.norm() * v12.norm());

        const Eigen::Vector3d v03 = tSlot->corners[3].worldCorner - tSlot->corners[0].worldCorner;
        const double cosValue2 = v03.dot(v01) / (v01.norm() * v03.norm());

        if(tSlot->id == debugParameter.targetSlotId) {
            std::cout<<"angle 1: "<<std::acos(cosValue1)*180.0f/M_PI<<" "<<"angle 2: "<<std::acos(cosValue2)*180.0f/M_PI<<"\n";
            std::cout<<"sum angle: "<<(0.5 * std::acos(cosValue1) + 0.5 * std::acos(cosValue2))*180.0f/M_PI<<"\n";
        }

        if(std::fabs(std::acos(cosValue1) - std::acos(cosValue2)) < (double)(17.0f/180.0f)*M_PI) {
            float sumAngle = 0.5 * std::acos(cosValue1) + 0.5 * std::acos(cosValue2);
            if( (sumAngle < (double)(70.0f/180.0f)*M_PI) 
             || (sumAngle > (double)(110.0f/180.0f)*M_PI) ) {
                tSlot->type = FusionSlot::Type::INCLINED;
                if(tSlot->id == debugParameter.targetSlotId) {
                    std::cout<<"set slot"<<tSlot->id<<"as inclined"<<"\n";
                }
            }
        }
        tSlot->type = (tSlot->type == FusionSlot::Type::INCLINED)?tSlot->type:FusionSlot::Type::UNKNOWN;
    }

    for(auto& tSlot : trackSlots) {
        if(tSlot->finalStatus == FusionSlot::Status::AVAILABLE) {
            calcWithEgoWheelStop(tSlot);
            calcWithOtherSlot(tSlot, trackSlots);
            calcWithFSStruct(tSlot, frame.fs_);
            
            finalOpenLineSelect(tSlot);
        }
        else {
            tSlot->openLineIndex = 4;
            //EstimateOpenLinePosition(tSlot);
        }
    }

    for(auto& tSlot : trackSlots) {
        if( (tSlot->openLineIndex != 4)
         && (tSlot->openLineIndex != 0) )
        {
            int rotateStep = 4 - tSlot->openLineIndex;
            tSlot->CornersAntiClockwiseRotate(rotateStep);
            for(auto& perceptSlot : tSlot->relativeSlots)
            {
                perceptSlot->CornersAntiClockwiseRotate(rotateStep);
                uint8_t indexDiffAfterRotate = perceptSlot->indexDiffWithAlignTrackSlotInAntiClockwise - (tSlot->openLineIndex);
                perceptSlot->indexDiffWithAlignTrackSlotInAntiClockwise = (0 > (indexDiffAfterRotate))?(indexDiffAfterRotate + 4):indexDiffAfterRotate;
            }
        }

        // const Eigen::Vector3d v01 = tSlot->corners[1].worldCorner - tSlot->corners[0].worldCorner;
        // const Eigen::Vector3d v12 = tSlot->corners[2].worldCorner - tSlot->corners[1].worldCorner;
        // const double cosValue1 = v12.dot(v01) / (v01.norm() * v12.norm());

        // const Eigen::Vector3d v03 = tSlot->corners[3].worldCorner - tSlot->corners[0].worldCorner;
        // const double cosValue2 = v03.dot(v01) / (v01.norm() * v03.norm());

        // if(std::fabs(std::acos(cosValue1) - std::acos(cosValue2)) < (double)(17.0f/180.0f)*M_PI) {
        //     float sumAngle = 0.5 * std::acos(cosValue1) + 0.5 * std::acos(cosValue2);
        //     if( (sumAngle < (double)(70.0f/180.0f)*M_PI) 
        //      || (sumAngle > (double)(110.0f/180.0f)*M_PI) ) {
        //         tSlot->type = FusionSlot::Type::INCLINED;
        //         if((tSlot->id == debugParameter.debugSlotId)||(tSlot->id == debugParameter.debugSlotId2)) {
        //             std::cout<<"set slot"<<tSlot->id<<"as inclined"<<"\n";
        //         }
        //     }
        // }
        // tSlot->type = (tSlot->type == FusionSlot::Type::INCLINED)?tSlot->type:FusionSlot::Type::UNKNOWN;
    }
}

void PldFusion::calcWithEgoWheelStop(const FusionSlot::Ptr targetSlot)
{
    if(targetSlot->block_.type == FusionSlot::BlockPoint::Type::NO_BLOCK) {
        return;
    }
    
    for(int i = 0; i < 2; i++) {
        Fusion::CTrackLine::Ptr egoLine = targetSlot->lines_[i];
        float egoDist = egoLine->calcOrthDistOfWorldPointAbs(targetSlot->block_.worldPoint);

        Fusion::CTrackLine::Ptr oppositeLine = targetSlot->lines_[(i + 2)%4];
        float opposideDist = oppositeLine->calcOrthDistOfWorldPointAbs(targetSlot->block_.worldPoint);

        if((opposideDist - egoDist) > 1.5f) {
            egoLine->couldBeingAsOpenLine = Fusion::CTrackLine::OpenLineStatus::occupiedByWheelStop;
        }
        else if((opposideDist - egoDist) < -1.5f) {
            oppositeLine->couldBeingAsOpenLine = Fusion::CTrackLine::OpenLineStatus::occupiedByWheelStop;
        }
    }
}


void PldFusion::calcWithFSStruct(
    const FusionSlot::Ptr slot,
    const FSStruct::Ptr fsStruct)
{
    if(fsStruct == nullptr) {
        return;
    }
    Eigen::Vector3d middlePoint;
    middlePoint << slot->middlePoint.worldCorner[0], slot->middlePoint.worldCorner[1], 0.0;

    for(int i = 0; i <  fsStruct->outerObjects_.size(); i++) {
        for(int j = 0; j < fsStruct->outerObjects_[i].size(); j++) {
            if((middlePoint - fsStruct->outerObjects_[i][j].worldCorner).norm() > 8.0) {
                continue;
            }
            else {
                PldFusion::calcWithFSPoint(slot, fsStruct->outerObjects_[i][j].worldCorner);
            }
        }
    }
}


void PldFusion::calcWithFSPoint(const FusionSlot::Ptr slot, const Eigen::Vector3d fsPoint)
{
    for(int i = 0; i < 4; i++) {
        Fusion::CTrackLine::Ptr line = slot->lines_[i];

        Fusion::CTrackLine::Ptr startConnectLine = slot->lines_[(i == 0)?3:(i - 1)];
        Fusion::CTrackLine::Ptr endConnectLine = slot->lines_[(i + 1)%4];

        if(line->couldBeingAsOpenLine != Fusion::CTrackLine::OpenLineStatus::unknown) {
            continue;
        }
        float tau = line->calcTauOfOrthProj(fsPoint);
        float startTau = line->getStartPointTau();
        float endTau = line->getEndPointTau();

        if(isValuesInOrder(startTau, tau, endTau)
        && ( (std::fabs(tau - startTau)/line->getLength() > 0.2f) 
          && (std::fabs(endTau - tau)/line->getLength() > 0.2f) ) 
        && (line->calcOrthDistOfWorldPointAbs(fsPoint) < 1.5f) ) {

            // if(slot->id == 3) {
            //     std::cout<<"fs line sp: "<<line->getStartPoint()[0]<<" "<<line->getStartPoint()[1]<<"\n";
            //     std::cout<<"fs line ep: "<<line->getEndPoint()[0]<<" "<<line->getEndPoint()[1]<<"\n";
            // }
            if(std::fabs(tau - startTau) < std::fabs(endTau - tau)) {
                if((startConnectLine->calcOrthDistOfWorldPointAbs(fsPoint) > 0.17f*line->getLength())
                && (startConnectLine->isPointOnLeftSideOfUnitVector(fsPoint) )) {
                    line->couldBeingAsOpenLine = Fusion::CTrackLine::OpenLineStatus::occupiedByFs;
                }
            }
            else {
                if((endConnectLine->calcOrthDistOfWorldPointAbs(fsPoint) > 0.17f*line->getLength())
                && (endConnectLine->isPointOnLeftSideOfUnitVector(fsPoint) )) {
                    line->couldBeingAsOpenLine = Fusion::CTrackLine::OpenLineStatus::occupiedByFs;
                }
            }

            // if((slot->id == debugParameter.debugSlotId)||(slot->id == debugParameter.debugSlotId2)) {
            //     if( (i == 0) || (line->couldBeingAsOpenLine = Fusion::CTrackLine::OpenLineStatus::occupiedByFs) )  {
            //         std::cout<<"slot id: "<<slot->id<<" line 0 fs point found"<<"\n";
            //         std::cout<<"fs: "<<fsPoint[0]<<", "<<fsPoint[1]
            //             <<" start tau: "<<startTau<<" tau: "<<tau
            //             <<" end tau: "<<endTau<<" orthDist: "<<line->calcOrthDistOfWorldPointAbs(fsPoint)<<"\n";
            //     }
            // }


        }
    }
}

void PldFusion::calcWithOtherSlot(
    const FusionSlot::Ptr targetSlot,
    const std::list<FusionSlot::Ptr> trackSlots)
{
    for(auto& tSlot : trackSlots) {
        if(targetSlot == tSlot) {
            continue;
        }
        
        if((targetSlot->middlePoint.worldCorner - tSlot->middlePoint.worldCorner).norm() > 8.0f) {
            continue;
        }

        PldFusion::neighborLinesDispatch(
            targetSlot, 
            tSlot);
    }
}

void PldFusion::neighborLinesDispatch(
    const FusionSlot::Ptr slotA, 
    const FusionSlot::Ptr slotB)
{
    for(int i = 0; i < 4; i++) {
        Fusion::CTrackLine::Ptr lineA = slotA->lines_[i];
        if(lineA->couldBeingAsOpenLine != Fusion::CTrackLine::OpenLineStatus::unknown) {
            continue;
        }
        Fusion::CTrackLine::Ptr oppositeLineA = slotA->lines_[(i + 2)%4];
        for(int j = 0; j < 4; j++) {

            Fusion::CTrackLine::Ptr lineB = slotB->lines_[j];
            Fusion::CTrackLine::Ptr oppositeLineB = slotB->lines_[(j + 2)%4];
            PldFusion::neighborLineHandle(lineA, oppositeLineA, lineB, oppositeLineB);

            if(lineA->couldBeingAsOpenLine != Fusion::CTrackLine::OpenLineStatus::unknown) {
                break;
            }
        }
    }
}

void PldFusion::neighborLineHandle(
    const Fusion::CTrackLine::Ptr lineA,
    const Fusion::CTrackLine::Ptr oppositeLineA,
    const Fusion::CTrackLine::Ptr lineB,
    const Fusion::CTrackLine::Ptr oppositeLineB)
{
    if(std::fabs(Fusion::calAngleBetweenTwoUnitVector(lineA->getDefaultUnitVec(), lineB->getDefaultUnitVec()))
       < (float)(3*M_PI/4.0f)) {
            return;
    }

    Eigen::Vector3d middleA = (lineA->getStartPoint() + lineA->getEndPoint())/2.0f;
    Eigen::Vector3d middleB = (lineB->getStartPoint() + lineB->getEndPoint())/2.0f;
    if(lineA->isPointOnLeftSideOfUnitVector(middleB) 
    && oppositeLineA->isPointOnLeftSideOfUnitVector(middleB)) {
        if((lineA->calcOrthDistOfWorldPointAbs(middleB) > 1.5) 
        && (oppositeLineA->calcOrthDistOfWorldPointAbs(middleB) > 1.5) ) {
            return;
        }
    }

    if(lineB->isPointOnLeftSideOfUnitVector(middleA) 
    && oppositeLineB->isPointOnLeftSideOfUnitVector(middleA)) {
        if((lineB->calcOrthDistOfWorldPointAbs(middleA) > 1.5) 
        && (oppositeLineB->calcOrthDistOfWorldPointAbs(middleA) > 1.5) ) {
            return;
        }
    }
    
    if( (lineA->calcOrthDistOfWorldPointAbs(lineB->getStartPoint()) < 2.2)
     && (lineA->calcOrthDistOfWorldPointAbs(lineB->getEndPoint()) < 2.2) )
    {
        float l_overlapsLength_f(0.0f);
        bool l_isOverlap_b = lineA->isOverlap(lineB->getStartPoint(), lineB->getEndPoint(), l_overlapsLength_f);
        
        float l_overlapRatioA_f = (float)(l_overlapsLength_f/lineA->getLength());
        if(l_overlapRatioA_f > 0.27) {
            lineA->couldBeingAsOpenLine = Fusion::CTrackLine::OpenLineStatus::occupiedByNeighborSlot;
            lineB->couldBeingAsOpenLine = Fusion::CTrackLine::OpenLineStatus::occupiedByNeighborSlot;
        }
        float l_overlapRatioB_f = (float)(l_overlapsLength_f/lineB->getLength());
        if(l_overlapRatioB_f > 0.27) {
            lineA->couldBeingAsOpenLine = Fusion::CTrackLine::OpenLineStatus::occupiedByNeighborSlot;
            lineB->couldBeingAsOpenLine = Fusion::CTrackLine::OpenLineStatus::occupiedByNeighborSlot;
        }
    }
}

void PldFusion::finalOpenLineSelect(const FusionSlot::Ptr targetSlot)
{
    std::array<float, 4> debugInfo;

    std::array<Fusion::CTrackLine::Ptr, 2> lines;
    std::array<float, 2> costs;
    for(int i = 0; i < 2; i++) {
        //std::cout<<"line "<<i;
        
        //for targetSlot->lines_[i]
        float openLineCostEgo = PldFusion::calcLineCostAsOpenLine(
            targetSlot->corners[i].baseCorner,
            targetSlot->corners[(i + 1)%4].baseCorner);

        debugInfo[i] = openLineCostEgo;

        //std::cout<<"line "<<(i + 2)%4;

        //for targetSlot->lines_[(i + 2)%4]
        float openLineCostOpposide = PldFusion::calcLineCostAsOpenLine(
            targetSlot->corners[(i + 2)%4].baseCorner,
            targetSlot->corners[(i + 3)%4].baseCorner);
        
        debugInfo[i + 2] = openLineCostOpposide;


        Fusion::CTrackLine::Ptr lowerCostLine = nullptr;
        Fusion::CTrackLine::Ptr higherCostLine = nullptr;
        float lowerCost;
        float higherCost;
        if(openLineCostEgo < openLineCostOpposide) {
            lowerCostLine = targetSlot->lines_[i];
            lowerCost = openLineCostEgo;

            higherCostLine = targetSlot->lines_[(i + 2)%4];
            higherCost = openLineCostOpposide;
        }
        else {
            lowerCostLine = targetSlot->lines_[(i + 2)%4];
            lowerCost = openLineCostOpposide;

            higherCostLine = targetSlot->lines_[i];
            higherCost = openLineCostEgo;
        }
        if(PldFusion::pairLinesHandle(lowerCostLine, higherCostLine) == 1) {
            lines[i] = lowerCostLine;
            costs[i] = lowerCost;
        }
        else {
            lines[i] = higherCostLine;
            costs[i] = higherCost;
        }
    }

    // if((targetSlot->id == debugParameter.debugSlotId)||(targetSlot->id == debugParameter.debugSlotId2)) {

    //     std::cout<<"slot id: "<<targetSlot->id<<"\n";
    //     for(int i  = 0; i < 4; i++) {
    //         std::cout<<"line "<<i<<" cost: "<<debugInfo[i]<<" occupied status: ";
            
    //         if(targetSlot->lines_[i]->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::unknown) {
    //             std::cout<<"not occupied";
    //         }
    //         else if(targetSlot->lines_[i]->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::occupiedByWheelStop) {
    //             std::cout<<"by wheel stop";
    //         }
    //         else if(targetSlot->lines_[i]->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::occupiedByNeighborSlot) {
    //             std::cout<<"by neighbor";
    //         }
    //         else if(targetSlot->lines_[i]->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::occupiedByFs) {
    //             std::cout<<"by fs";
    //         }
    //         else if(targetSlot->lines_[i]->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::oppositeOccupiedByNeighborSlot) {
    //             std::cout<<"opposite by neighbor";
    //         }

    //         std::cout<<"\n";
    //     }
    // }
    
    PldFusion::finalBattle(lines, costs, targetSlot);

}

void PldFusion::finalBattle(
    std::array<Fusion::CTrackLine::Ptr, 2>& lines,
    std::array<float, 2> costs,
    const FusionSlot::Ptr targetSlot)
{
    Fusion::CTrackLine::Ptr selectedLine = nullptr;

    if(costs[0] > costs[1]) {
        std::swap(costs[0], costs[1]);
        std::swap(lines[0], lines[1]);
    }

    if(lines[0]->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::unknown) {
        selectedLine = lines[0];
    }
    else {
        if(lines[1]->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::unknown) {
            selectedLine = lines[1];
        }
    }

    // if(targetSlot->id == 3) {
    //     std::cout<<"line 0 ocupied by: "<<lines[0]->couldBeingAsOpenLine<<"\n";
    //     std::cout<<"line 0 sp: "<<lines[0]->getStartPoint()[0]<<" "<<lines[0]->getStartPoint()[1]<<"\n";
    //     std::cout<<"line 0 ep: "<<lines[0]->getEndPoint()[0]<<" "<<lines[0]->getEndPoint()[1]<<"\n";


    //     std::cout<<"line 1 ocupied by: "<<lines[1]->couldBeingAsOpenLine<<"\n";
    //     std::cout<<"line 1 sp: "<<lines[1]->getStartPoint()[0]<<" "<<lines[1]->getStartPoint()[1]<<"\n";
    //     std::cout<<"line 1 ep: "<<lines[1]->getEndPoint()[0]<<" "<<lines[1]->getEndPoint()[1]<<"\n";
    // }


    if( (targetSlot->type == FusionSlot::Type::INCLINED) 
     && (lines[0]->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::unknown)
     && (lines[1]->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::unknown) ) {
        if(lines[0]->getLength() > lines[1]->getLength()) {
            selectedLine = lines[1];
        }
        else {
            selectedLine = lines[0];
        }
    }

    targetSlot->findOpenLineInSlot(selectedLine);

    if( (selectedLine != nullptr) && (targetSlot->block_.type != FusionSlot::BlockPoint::Type::NO_BLOCK) ) {
        auto egoLine = targetSlot->lines_[targetSlot->openLineIndex];
        auto opposideLine = targetSlot->lines_[(targetSlot->openLineIndex + 2)%4];
        if( (egoLine->isPointOnRightSideOfUnitVector(targetSlot->block_.worldPoint) 
          && (!opposideLine->isPointOnRightSideOfUnitVector(targetSlot->block_.worldPoint)))
         || ((!egoLine->isPointOnRightSideOfUnitVector(targetSlot->block_.worldPoint))
          && opposideLine->isPointOnRightSideOfUnitVector(targetSlot->block_.worldPoint) ) ) {
            if(egoLine->calcOrthDistOfWorldPointAbs(targetSlot->block_.worldPoint)
             > opposideLine->calcOrthDistOfWorldPointAbs(targetSlot->block_.worldPoint) ) {
                //do nothing
                //std::cout<<"keep"<<"\n";
            }
            else {
                targetSlot->openLineIndex = (targetSlot->openLineIndex + 2)%4;
                //std::cout<<"opposide"<<"\n";
            }
        }
    }

    // if((targetSlot->id == debugParameter.debugSlotId)||(targetSlot->id == debugParameter.debugSlotId2)) {
    //    std::cout<<"slot id: "<<targetSlot->id<<" open index: "<<targetSlot->openLineIndex<<"\n";
    // }

    
}


float PldFusion::calcLineCostAsOpenLine(
    const Eigen::Vector3d startPoint,
    const Eigen::Vector3d endPoint)
{
    Fusion::CTrackLine::Ptr baseLine = std::make_shared<Fusion::CTrackLine>(startPoint, endPoint);
    
    float tau = baseLine->calcTauOfOrthProj(Eigen::Vector3d(2.0f, 0.0f, 0.0f));
    if(Fusion::isValuesInOrder(baseLine->getStartPointTau() ,tau, baseLine->getEndPointTau())) {
        tau = 0;
    }
    else {
        tau = std::min(std::fabs(baseLine->getStartPointTau() - tau), std::fabs(tau - baseLine->getEndPointTau()));
    }

    float value = baseLine->calcOrthDistOfWorldPointAbs(Eigen::Vector3d(2.0f, 0.0f, 0.0f)) + tau;
    //((startPoint + endPoint)/2.0f - Eigen::Vector3d(3.0f, 0.0f, 0.0f)).norm(); //smaller, more nearby vehicle, more possiable as openLine
    //std::cout<<" dist cost: "<<value;

    Eigen::Vector3d baseUnitVector = 
        (Eigen::Vector3d(endPoint[0], endPoint[1], 0.0f) - Eigen::Vector3d(startPoint[0], startPoint[1], 0.0f));
    baseUnitVector.normalize();
    
    float angleValue = Fusion::calAngleBetweenTwoUnitVector(baseUnitVector, Eigen::Vector3d(1.0, 0.0, 0.0));
    //std::cout<<" angle: "<<angleValue;
    angleValue = std::fabs(angleValue);

    //angleValue [0, 0.5*M_PI]    
    angleValue = (angleValue > 0.5f*M_PI)?(M_PI - angleValue):angleValue;

    angleValue = 5.5f*angleValue/(0.5f*M_PI);   //value smaller, more parallel, more possiable as openLine
    //std::cout<<" angle cost: "<<angleValue<<"\n";
    value += angleValue;

    return value;
}

//return value:  1 - lowerCostLine  2 - higherCostLine
int PldFusion::pairLinesHandle(
    const Fusion::CTrackLine::Ptr lowerCostLine, 
    const Fusion::CTrackLine::Ptr higherCostLine)
{
    if(lowerCostLine->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::occupiedByWheelStop) {
        return 2;
    }
    else {
        return 1;
    }

    // if( (lowerCostLine->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::unknown) 
    //  || (lowerCostLine->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::occupiedByFs)) {
    //     return 1;
    // }

    // if(lowerCostLine->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::occupiedByWheelStop) {
    //     return 2;
    // }

    // if(lowerCostLine->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::occupiedByNeighborSlot) {
    //     if(higherCostLine->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::unknown) {
    //         higherCostLine->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::oppositeOccupiedByNeighborSlot;
    //     }

    //     if( (higherCostLine->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::occupiedByNeighborSlot)
    //      && (higherCostLine->couldBeingAsOpenLine == Fusion::CTrackLine::OpenLineStatus::occupiedByWheelStop) ) {
    //         return 1;
    //     }
    //     else {
    //         return 2;
    //     }
    // }
}




}
}