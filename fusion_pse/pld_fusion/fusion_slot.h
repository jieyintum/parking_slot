#ifndef FUSION_PSE_FUSIONSLOT_H_
#define FUSION_PSE_FUSIONSLOT_H_

#include "inner_msgs/percept_slot.h"
#include "inner_msgs/fusion_slots.h"
#include "inner_msgs/uss_msgs.h"
#include "utils/fixed_size_deque.h"
#include "line_process/line_common_function.hpp"
#include "line_process/trackLine.h"


namespace Fusion {
namespace PSE {

struct debugParams {
    float frontAxelForParallel = 0.0f;
    float rearAxelForParallel = 0.0f;

    float frontAxelForVertical = 0.0f;
    float rearAxelForVertical = 0.0f;

    int targetSlotId = 0;
    
    #if DEBUG_MODE
    float showMarkerScale = 0.02;
    
    int debugSlotId = 6;
    int debugSlotId2 = 6;

    bool verticalAsRectangleSwitch = true;
    double maxBasePointUpdateCost = 8.5;
    double minBasePointUpdateCost = 1.0;
    double maxBuffer = 0.15;
    double addBufferXThreshold = 3.0;
    double depthCornerRatio = 2.0;
    double baseCostRatio = 4.0;
    bool addBuffer = true;

    bool showOpenLine = false;
    bool isUSSDeleteOpen = true;
    double covErrorRatio = 0.5;

    bool isDepthsKeepWork = true;

    #endif
};


struct FusionSlot {
    using Ptr = std::shared_ptr<FusionSlot>;
    enum class SlotLoc : uint8_t {
        UNKNOWN = 0U,
        LEFT = 1U,
        RIGHT = 2U
    };

    enum class Type : uint8_t {
        UNKNOWN = 0U,
        HORIZONTAL = 1U,
        VERTICAL = 2U,
        INCLINED = 3U
    };

    enum class Status : uint8_t {
        DEFAULT = 0U,
        AVAILABLE,
        OCCUPIED
    };


    enum class ContributingSensor : uint8_t {
        DEFAULT = 0U,
        PURE_VISION = 1U,
        PURE_USS = 2U,
        VISON_USS_MERGED = 3U
    };


    struct EdgePoint {
        Eigen::Vector3d point;
        Eigen::Vector3d worldPoint;
        bool valid;
        int32_t lifeCounter;
    };

    struct SlotCorner {
        struct CornerError {
            double weight = 0.0;
            double cov = 0.0;
            double error = 0.0;
        };

        Eigen::Vector3d baseCorner = Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d worldCorner = Eigen::Vector3d(0.0, 0.0, 0.0);
        CornerError covError;
        double confidence = 1.0;

        double weightForBaseCorner = 0.0;
        bool isCornerAsTruePoint = false;
    };

    struct BlockPoint {
        enum class Type {   // 0:Default, without block, 1:predictive block, 2:detective block
            NO_BLOCK = 0,
            PREDICT,
            DETECTIVE
        };
        Eigen::Vector3d basePoint{0.0, 0.0, 0.0};
        Eigen::Vector3d worldPoint{0.0, 0.0, 0.0};
        Type type = Type::NO_BLOCK;
        double confidence = 1.0;
        double depth = 0.0;
    };

    struct AdjoinSlot {
        FusionSlot::Ptr leftSlot = nullptr;
        FusionSlot::Ptr rightSlot = nullptr;
        bool isProcess = false;
    };

    struct AlignInfo {
        FusionSlot::Ptr lastPartnerSlot = nullptr; //for uss/pld slot
        FusionSlot::Ptr lastResultSlot = nullptr; //for uss/pld slot
        
        FusionSlot::Ptr partnerSlot = nullptr; //for uss/pld slot
        FusionSlot::Ptr resultSlot = nullptr; //for uss/pld slot

        FusionSlot::Ptr ussSlot = nullptr; //for merged slot
        FusionSlot::Ptr pldSlot = nullptr; //for merged slot
    };

    struct matchedInfo{
        FusionSlot::Ptr matchedSlot = nullptr;
        int32_t numOfMatchedLines = 0;
        float distSumOfMatchedLines = 0.0f;
        bool isDispatched = false;


        bool operator > (const matchedInfo& info) const
        {
            if(numOfMatchedLines != info.numOfMatchedLines) {
                return (numOfMatchedLines > info.numOfMatchedLines);
            }
            else {
                return (distSumOfMatchedLines < info.distSumOfMatchedLines);
            }
        }
    
        bool operator < (const matchedInfo& info) const
        {
            if(numOfMatchedLines != info.numOfMatchedLines) {
                return (numOfMatchedLines < info.numOfMatchedLines);
            }
            else {
                return (distSumOfMatchedLines > info.distSumOfMatchedLines);
            }
        }
    };


    FusionSlot() = default;
    ~FusionSlot() = default;

public:

    FusionSlot(const PLD_DL_Result& slotIn, const Odometry& pose)
    {
        for (int i = 0; i < 4; ++i) {
            corners[i].baseCorner << slotIn.slot_GroundPts[i].CornerPot.x, slotIn.slot_GroundPts[i].CornerPot.y, 0.0;
            corners[i].confidence = slotIn.slot_GroundPts[i].CornerPot_conf;
            // corners[i].worldCorner = pose.orientation * corners[i].baseCorner + pose.translation;
        }

        confidence_ = slotIn.slot_confidence;

        if(0.5f < (float)slotIn.block_GroundPt.Block_conf) {
            block_.basePoint << slotIn.block_GroundPt.BlockPot.x, slotIn.block_GroundPt.BlockPot.y, 0.0;
            block_.confidence = slotIn.block_GroundPt.Block_conf;
            block_.type = static_cast<FusionSlot::BlockPoint::Type>(slotIn.block_GroundPt.Block_type);
        }
        else {
            block_.basePoint << 0.0, 0.0, 0.0;
            block_.confidence = 0.0;
            block_.type = FusionSlot::BlockPoint::Type::NO_BLOCK;
        }

        finalStatus = static_cast<FusionSlot::Status>(slotIn.slot_status);

        calcMiddlePointBase();

        TransferToWorld(pose);
        PreProcessDetectEdges();

        initOdom = pose;
    }

    void calcCornerCost(
        const float ratio,
        const Type TrackSlotType,
        const bool addBuffer,
        const float minBasePointUpdateCost,
        const float maxBasePointUpdateCost,
        const float maxBuffer,
        const float addBufferXThreshold,
        const float depthCornerRatio,
        const Odometry& odom)
    {
        double baseCost = calcAngleCost(ratio);

        for(int i = 0; i < 4; i++) {
            corners[i].weightForBaseCorner = std::sin(baseCost)*std::fabs(corners[i].baseCorner[0] - 2.5f);// baseCost + std::min(std::fabs(corners[i].baseCorner[0] - 2.0f), std::fabs(corners[i].baseCorner[1]));
            //corners[i].weightForBaseCorner = baseCost - corners[i].baseCorner[0];
            corners[i].weightForBaseCorner = 
                ((TrackSlotType == Type::HORIZONTAL) 
              && ((std::fabs(corners[i].baseCorner[0] - 1.0f) > 6) 
               || (std::fabs(corners[i].baseCorner[1]) > 6)))?100:corners[i].weightForBaseCorner;
        }

        if(addBuffer == false) {
            return;
        }

        if(TrackSlotType != Type::HORIZONTAL) {
            std::array<Fusion::CTrackLine::Ptr, 4U> egoLines_;
            for (size_t i = 0U; i < 4U; ++i) {
                if(egoLines_[i] == nullptr) {
                    egoLines_[i] = std::make_shared<Fusion::CTrackLine>();
                }
                egoLines_[i]->initWithTwoWorldPoints(corners[i].baseCorner, corners[(i + 1)%4U].baseCorner);
            }

            bool moveToP1 = false;
            bool moveToP3 = false;
            if(corners[0].weightForBaseCorner < corners[1].weightForBaseCorner) {
                moveToP1 = true;
                moveToP3 = false;  
            }
            else {
                moveToP1 = false;
                moveToP3 = true;
            }

            for(int i = 0; i < 2; i++) {
                if(corners[i].baseCorner[0] > (float)addBufferXThreshold) {
                    continue;
                }

                float moveBuffer = (float)maxBuffer;
                if(corners[i].weightForBaseCorner > maxBasePointUpdateCost) {
                    //
                }
                else if( (corners[i].weightForBaseCorner < maxBasePointUpdateCost) 
                && (corners[i].weightForBaseCorner > minBasePointUpdateCost) ) {
                    moveBuffer = moveBuffer * 
                      (corners[i].weightForBaseCorner - minBasePointUpdateCost)/(maxBasePointUpdateCost- minBasePointUpdateCost);
                }
                else {
                    moveBuffer = 0.0f;
                    continue;
                }


                float newTauEgo;
                float oldTauEgo = (i == 0)?egoLines_[0]->getStartPointTau():egoLines_[0]->getEndPointTau();
                if( ((moveToP1 == true) && (egoLines_[0]->getIsUnitVectorInversed() == false))
                 || ((moveToP1 == false) && (egoLines_[0]->getIsUnitVectorInversed() == true)) ) {
                    newTauEgo = oldTauEgo - moveBuffer;
                }
                else {
                    newTauEgo = oldTauEgo + moveBuffer;
                }
                corners[i].baseCorner = egoLines_[0]->calcWorldPointUsingTauValue(newTauEgo);

                int oppositeIndex = i + 2;
                float newTauOpposite;
                float oldTauOpposite = (oppositeIndex == 2)?egoLines_[2]->getStartPointTau():egoLines_[2]->getEndPointTau();
                if( ((moveToP3 == true) && (egoLines_[2]->getIsUnitVectorInversed() == false))
                 || ((moveToP3 == false) && (egoLines_[2]->getIsUnitVectorInversed() == true)) ) {
                    newTauOpposite = oldTauOpposite - depthCornerRatio*moveBuffer;
                }
                else {
                    newTauOpposite = oldTauOpposite + depthCornerRatio*moveBuffer;
                }
                corners[oppositeIndex].baseCorner = egoLines_[2]->calcWorldPointUsingTauValue(newTauOpposite);
            }
            TransferToWorld(odom);
        }
    }

    double calcAngleCost(const float ratio) {
        Eigen::Vector3d uv = corners[0].baseCorner - corners[3].baseCorner;
        uv += corners[1].baseCorner - corners[2].baseCorner;
        uv.normalize();

        float angleValue = Fusion::calAngleBetweenTwoUnitVector(uv, Eigen::Vector3d(1.0, 0.0, 0.0));
        //std::cout<<" angle: "<<angleValue;
        angleValue = std::fabs(angleValue);

        //angleValue [0, 0.5*M_PI]    
        angleValue = (angleValue > 0.5f*M_PI)?(M_PI - angleValue):angleValue;

        //angleValue = ratio*angleValue/(0.5f*M_PI);   //value smaller, more parallel, more possiable as openLine

        return angleValue;
    }

    FusionSlot(const UssSlot::Ptr& ussSlot, const Odometry& pose)
    {
        for (uint32_t j = 0U; j < 2U; ++j) {
            edges_[j].point << 0.0, 0.0, 0.0;
            edges_[j].valid = false;
        }
        confidence_ = 0.0;

        block_.basePoint << 0.0, 0.0, 0.0;
        block_.confidence = 0.0;
        block_.type = FusionSlot::BlockPoint::Type::NO_BLOCK;
        finalStatus = Status::AVAILABLE;

        corners[0].baseCorner = ussSlot->p0;
        corners[0].confidence = 0.0;

        corners[1].baseCorner = ussSlot->p1;
        corners[1].confidence = 0.0;

        corners[2].baseCorner = ussSlot->p2;
        corners[2].confidence = 0.0;

        corners[3].baseCorner = ussSlot->p3;
        corners[3].confidence = 0.0;
        
        middlePoint.baseCorner << 0.0, 0.0, 0.0;
        for(uint8_t i = 0; i < 4; i++) {
            middlePoint.baseCorner += corners[i].baseCorner;
        }

        middlePoint.baseCorner[0] =  (double)middlePoint.baseCorner[0]/4.0f;
        middlePoint.baseCorner[1] =  (double)middlePoint.baseCorner[1]/4.0f;
        
        TransferToWorld(pose);

        locForUssSlot =static_cast<SlotLoc>(ussSlot->loc);
        type = static_cast<Type>(ussSlot->type);

        initLines();

        originP0Tau = lines_[0]->getStartPointTau();
        originP1Tau = lines_[0]->getEndPointTau();
        middleTau = 0.5f*(originP0Tau + originP1Tau);
    }

    FusionSlot(const FusionSlot& inSlot)
    {
        corners = inSlot.corners;
        edges_ = inSlot.edges_;
        block_ = inSlot.block_;
        confidence_ = inSlot.confidence_;
        finalStatus = inSlot.finalStatus;
        isOccupiedByVeh = inSlot.isOccupiedByVeh;
        isOccupiedByLocker = inSlot.isOccupiedByLocker;
        id = inSlot.id;
        isReverse = inSlot.isReverse;
        type =  inSlot.type;
        width = inSlot.width;
        length = inSlot.length;
        angle = inSlot.angle;
        initOdom = inSlot.initOdom;
    }

    FusionSlot(const int isAsObserveRange, const Fusion::veh_params& vehicleParams)
    {
        if(isAsObserveRange == 1) {
            std::cout<<"init slot type 1"<<"\n";
            corners[0].baseCorner[0] = vehicleParams.g_maxObserveVehicleCoordX_f;
            corners[0].baseCorner[1] = vehicleParams.g_maxObserveVehicleCoordY_f;
            corners[0].baseCorner[2] = 0.0f;

            corners[1].baseCorner[0] = vehicleParams.g_minObserveVehicleCoordX_f;
            corners[1].baseCorner[1] = vehicleParams.g_maxObserveVehicleCoordY_f;
            corners[1].baseCorner[2] = 0.0f;

            corners[2].baseCorner[0] = vehicleParams.g_minObserveVehicleCoordX_f;
            corners[2].baseCorner[1] = vehicleParams.g_minObserveVehicleCoordY_f;
            corners[2].baseCorner[2] = 0.0f;

            corners[3].baseCorner[0] = vehicleParams.g_maxObserveVehicleCoordX_f;
            corners[3].baseCorner[1] = vehicleParams.g_minObserveVehicleCoordY_f;
            corners[3].baseCorner[2] = 0.0f;
        }
        else if(isAsObserveRange == 0){
            std::cout<<"init slot type 0"<<"\n";
            corners[0].baseCorner[0] = vehicleParams.g_maxVehicleCoordX_f;
            corners[0].baseCorner[1] = vehicleParams.g_maxVehicleCoordY_f;
            corners[0].baseCorner[2] = 0.0f;

            corners[1].baseCorner[0] = vehicleParams.g_minVehicleCoordX_f;
            corners[1].baseCorner[1] = vehicleParams.g_maxVehicleCoordY_f;
            corners[1].baseCorner[2] = 0.0f;

            corners[2].baseCorner[0] = vehicleParams.g_minVehicleCoordX_f;
            corners[2].baseCorner[1] = vehicleParams.g_minVehicleCoordY_f;
            corners[2].baseCorner[2] = 0.0f;

            corners[3].baseCorner[0] = vehicleParams.g_maxVehicleCoordX_f;
            corners[3].baseCorner[1] = vehicleParams.g_minVehicleCoordY_f;
            corners[3].baseCorner[2] = 0.0f;
            initLines(true);
        }
        else if(isAsObserveRange == 2) {
            std::cout<<"init slot type 2"<<"\n";
            corners[0].baseCorner[0] = vehicleParams.g_vehicleModelMaxX;
            corners[0].baseCorner[1] = vehicleParams.g_vehicleModelMaxY;
            corners[0].baseCorner[2] = 0.0f;

            corners[1].baseCorner[0] = vehicleParams.g_vehicleModelMinX;
            corners[1].baseCorner[1] = vehicleParams.g_vehicleModelMaxY;
            corners[1].baseCorner[2] = 0.0f;

            corners[2].baseCorner[0] = vehicleParams.g_vehicleModelMinX;
            corners[2].baseCorner[1] = vehicleParams.g_vehicleModelMinY;
            corners[2].baseCorner[2] = 0.0f;

            corners[3].baseCorner[0] = vehicleParams.g_vehicleModelMaxX;
            corners[3].baseCorner[1] = vehicleParams.g_vehicleModelMinY;
            corners[3].baseCorner[2] = 0.0f;
            initLines(true);
        }
    }


    void TransferToWorld(const Odometry& odom)
    {
        //std::cout<<"translation: "<<odom.translation[0]<<","<<odom.translation[1]<<","<<odom.translation[2]<<"\n";
        for (size_t i = 0U; i < 4U; ++i) {
            //std::cout<<"baseCorner: "<<corners[i].baseCorner[0]<<","<<corners[i].baseCorner[1]<<","<<corners[i].baseCorner[2]<<"\n";
            corners[i].worldCorner = odom.orientation * corners[i].baseCorner + odom.translation;
            // std::cout<<"worldCorner: "<<corners[i].worldCorner[0]<<","<<corners[i].worldCorner[1]<<","<<corners[i].worldCorner[2]<<"\n";
            // Eigen::Vector3d newBase = odom.orientation.inverse() * (corners[i].worldCorner - odom.translation);
            // std::cout<<"baseCorner after transfer: "<<newBase[0]<<","<<newBase[1]<<","<<newBase[2]<<"\n";
        }
        for (size_t i = 0U; i < 2U; ++i) {
            edges_[i].worldPoint = odom.orientation * edges_[i].point + odom.translation;
        }
        block_.worldPoint = odom.orientation * block_.basePoint + odom.translation;
        middlePoint.worldCorner = odom.orientation * middlePoint.baseCorner + odom.translation;
    }

    void TransferToBase(const Odometry& odom)
    {
        for (size_t i = 0U; i < 4U; ++i) {
            corners[i].baseCorner = odom.orientation.inverse() * (corners[i].worldCorner - odom.translation);
        }
        for (size_t i = 0U; i < 2U; ++i) {
            edges_[i].point = odom.orientation.inverse() * (edges_[i].worldPoint - odom.translation);
        }
        block_.basePoint = odom.orientation.inverse() * (block_.worldPoint - odom.translation);
        middlePoint.baseCorner = odom.orientation.inverse() * (middlePoint.worldCorner - odom.translation);
    }

    void CornersAntiClockwiseRotate(const int& step)
    {
        std::array<SlotCorner, 4U> oldCorners =  corners;
        for(int i = 0; i < 4; i++) {
            corners[(step + i)%4] = oldCorners[i];
        }
    }

    void PreProcessDetectEdges()
    {
        const double dist01 = (edges_[0].worldPoint - corners[1].worldCorner).norm();
        const double dist10 = (edges_[1].worldPoint - corners[0].worldCorner).norm();
        if (dist01 < 1.5 || dist10 < 1.5) {
            std::swap(edges_[0], edges_[1]);
        }
        const double dist00 = (edges_[0].worldPoint - corners[0].worldCorner).norm();
        edges_[0].valid &= (dist00 < 1.5);
        const double dist11 = (edges_[1].worldPoint - corners[1].worldCorner).norm();
        edges_[1].valid &= (dist11 < 1.5);
    }

    SlotMsg ToMsg(const int mode)
    {
        SingleSlot slotOut;
        slotOut.id = id;
        slotOut.isOccupied = (this->finalStatus == Status::OCCUPIED);
        slotOut.type = static_cast<uint8_t>(this->type);

        if(mode == 1) {
            slotOut.location = (static_cast<uint8_t>(this->locForUssSlot));
        }
        else {
            slotOut.location = (static_cast<uint8_t>(this->isReverse) + 1U);
        }

        slotOut.contributingSensors = static_cast<uint8_t>(sensor);
        if (slotOut.isOccupied) {
            slotOut.occupiedType = 3U;
            if(this->isOccupiedByVeh) {
                slotOut.occupiedType = 1U;
            }
            else if(this->isOccupiedByLocker) {
                slotOut.occupiedType = 2U;
                //std::cout<<"slot id: "<<id<<" is occupied by locker"<<"\n";
            }
        } else {
            slotOut.occupiedType = 0U;
        }

        slotOut.p0 = Eigen::Vector3d(this->corners[0].worldCorner[0], this->corners[0].worldCorner[1], 0.0);
        slotOut.p1 = Eigen::Vector3d(this->corners[1].worldCorner[0], this->corners[1].worldCorner[1], 0.0);
        slotOut.p2 = Eigen::Vector3d(this->corners[2].worldCorner[0], this->corners[2].worldCorner[1], 0.0);
        slotOut.p3 = Eigen::Vector3d(this->corners[3].worldCorner[0], this->corners[3].worldCorner[1], 0.0);

        slotOut.angle = angle;
        slotOut.width = width;
        slotOut.depth = length;

        slotOut.hasWheelStop = (this->block_.type != FusionSlot::BlockPoint::Type::NO_BLOCK);
        slotOut.wheelStopPoint = Eigen::Vector3d(this->block_.worldPoint[0], this->block_.worldPoint[1], 0.0);

        slotOut.releaseState = isParkable;

        return slotOut.ToMsg();
    }
 
    void initLines()
    {
        for (size_t i = 0U; i < 4U; ++i) {
            if(lines_[i] == nullptr) {
                lines_[i] = std::make_shared<Fusion::CTrackLine>();
            }
            lines_[i]->initWithTwoWorldPoints(corners[i].worldCorner, corners[(i + 1)%4U].worldCorner);
        }
    }

    void initLines(const std::array<float, 4U> lastFrameLengths)
    {
        for (size_t i = 0U; i < 4U; ++i) {
            if(lines_[i] == nullptr) {
                lines_[i] = std::make_shared<Fusion::CTrackLine>();
            }
            lines_[i]->initWithTwoWorldPoints(corners[i].worldCorner, corners[(i + 1)%4U].worldCorner);
            lines_[i]->lastFrameLength = lastFrameLengths[i];
        }
    }



    void initLines(const bool isUsingEgoPoint)
    {
        for (size_t i = 0U; i < 4U; ++i) {
            if(lines_[i] == nullptr) {
                lines_[i] = std::make_shared<Fusion::CTrackLine>();
            }
            lines_[i]->initWithTwoWorldPoints(corners[i].baseCorner, corners[(i + 1)%4U].baseCorner);
        }
    }

    void resetAlignInfo()
    {
        alignInfo.lastPartnerSlot = alignInfo.partnerSlot;
        alignInfo.lastResultSlot = alignInfo.resultSlot;
        
        alignInfo.partnerSlot = nullptr;
        alignInfo.resultSlot = nullptr;

        alignInfo.ussSlot = nullptr;
        alignInfo.pldSlot = nullptr;
    }
    
    void setAlignInfo(
        const FusionSlot::Ptr ussSourceSlot, 
        const FusionSlot::Ptr pldSourceSlot) //For Merged Slot
    {
        alignInfo.ussSlot = ussSourceSlot;
        alignInfo.pldSlot = pldSourceSlot;

        alignInfo.lastPartnerSlot = nullptr;
        alignInfo.lastResultSlot = nullptr;
        
        alignInfo.partnerSlot = nullptr;
        alignInfo.resultSlot = nullptr;
    }

    void setPtrToNullptr()
    {
        alignTrackSlot = nullptr;

        for(auto& rlslot_ : relativeSlots) {
            rlslot_ = nullptr;
        }

        resetEachCircleForPLDTracking();

        sourceSlot = nullptr;

        alignInfo.ussSlot = nullptr;
        alignInfo.pldSlot = nullptr;

        alignInfo.lastPartnerSlot = nullptr;
        alignInfo.lastResultSlot = nullptr;
        
        alignInfo.partnerSlot = nullptr;
        alignInfo.resultSlot = nullptr;
    }

    void calcMiddlePoint(const Odometry& odom) {
        TransferToBase(odom);
        middlePoint.baseCorner = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
        for(uint8_t i = 0; i < 4; i++) {
            middlePoint.baseCorner += corners[i].baseCorner;
        }
        middlePoint.baseCorner[0] = (double)middlePoint.baseCorner[0]/4.0f; 
        middlePoint.baseCorner[1] = (double)middlePoint.baseCorner[1]/4.0f;
        middlePoint.baseCorner[2] = 0.0f;

        middlePoint.worldCorner = odom.orientation * middlePoint.baseCorner + odom.translation;
    }

    void calcMiddlePointBase()
    {
        middlePoint.baseCorner = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
        for(uint8_t i = 0; i < 4; i++) {
            middlePoint.baseCorner += corners[i].baseCorner;
        }

        middlePoint.baseCorner[0] = (double)middlePoint.baseCorner[0]/4.0f; 
        middlePoint.baseCorner[1] = (double)middlePoint.baseCorner[1]/4.0f;
        middlePoint.baseCorner[2] = 0.0f;
    }

    void calcMiddlePointWorld()
    {
        middlePoint.worldCorner = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
        for(uint8_t i = 0; i < 4; i++) {
            middlePoint.worldCorner += corners[i].worldCorner;
        }
        middlePoint.worldCorner[0] = (double)middlePoint.worldCorner[0]/4.0f; 
        middlePoint.worldCorner[1] = (double)middlePoint.worldCorner[1]/4.0f;
        middlePoint.worldCorner[2] = (double)middlePoint.worldCorner[2]/4.0f;
    }

    void setIsParkable(const Fusion::PSE::debugParams params, const Odometry& odom)
    {
        if( (isParkable == false)
         && (sensor != ContributingSensor::PURE_USS) ) {

            float frontAxel(0.0f);
            float rearAxel(0.0f);

            if(type == Type::HORIZONTAL) {
                frontAxel = params.frontAxelForParallel;   // 5.0f;  // -2.5f;
                rearAxel = params.rearAxelForParallel;   //- 2.5f;  //-2.5f;
            }
            else {
                frontAxel = params.frontAxelForVertical;   //5.0f; //2.5f;
                rearAxel = params.rearAxelForVertical;    //-2.5f; //0.0f;
            }

            if(middlePoint.baseCorner[0] > frontAxel) {
                isEverBeforeFrontAxel = true;
            }
            else {
                isEverAfterFrontAxel = true;
            }

            if(middlePoint.baseCorner[0] > rearAxel) {
                isEverBeforeRearAxel = true;
            }
            else {
                isEverAfterRearAxel = true;
            }

            // std::cout<<"odom init translate: "<<initOdom.translation[0]<<" "<<initOdom.translation[1]<<" "<<initOdom.translation[2]<<"\n";

            if( (isMovingDistanceAfterInit == false)
             && (initOdom.translation - odom.translation).norm() > 1.3) {
                isMovingDistanceAfterInit = true;
            }

            if( (isMovingDistanceAfterInit == true)
              &&
                ( ((isEverBeforeFrontAxel == true) && (isEverAfterFrontAxel == true))
               || ((isEverBeforeRearAxel == true) && (isEverAfterRearAxel == true))
               || ((isEverAfterFrontAxel == true) && (isEverBeforeRearAxel == true))) ) {
                isParkable = true;
            }
        }
        else {
            isParkable = true;
        }
    }


    void replace(const FusionSlot::Ptr& newSlot)
    {
        uint16_t id_ = this->id;
        FusionSlot::AlignInfo alignInfo_ = this->alignInfo;
        FusionSlot::ContributingSensor tempSensor = this->sensor;
        bool tempIsParkable = this->isParkable;
        FusionSlot::Ptr tempSourceSlot = this->sourceSlot;

        *this = *newSlot;
        this->id = id_;
        this->alignInfo = alignInfo_;
        this->sensor = tempSensor;
        this->isParkable = tempIsParkable;
        this->sourceSlot = tempSourceSlot;
    }

    void addMatchedInfo(matchedInfo& newInfo)
    {
        for(int i = 0; i < matchedList.size(); i++) {
            if(newInfo > matchedList[i]) {
                matchedInfo backUp = matchedList[i];
                matchedList[i] = newInfo;
                newInfo = backUp;
            }
            else {
                //do nothing
            }
        }
        matchedList.push_back(newInfo);
    }


    void resetEachCircleForPLDTracking()
    {
        for(int i = 0; i < matchedList.size(); i++) {
            matchedList[i].matchedSlot = nullptr;
        }
        matchedList.clear();
        lowestCostInfo.matchedSlot = nullptr; //for tracked & detected slots
        for(int i = 0; i < 4; i++) {
            if(lines_[i] != nullptr) {
                lines_[i]->couldBeingAsOpenLine = Fusion::CTrackLine::OpenLineStatus::unknown;
            }
        }
    }

    void findOpenLineInSlot(const Fusion::CTrackLine::Ptr selectedLine)
    {
        if(selectedLine == nullptr) {
            finalStatus = Status::OCCUPIED;
            openLineIndex = 4;
            return;
        }

        for(int i = 0; i < 4; i++) {
            if(selectedLine == lines_[i]) {
                openLineIndex = i;
                return;
            }
        }

        finalStatus = Status::OCCUPIED;
        openLineIndex = 4;


        //std::cout<<"slot id:"<<id<<" selected line in valid"<<"\n";
    }

    bool getIsLengthTooShort() 
    {
        for(int i = 0; i < 4; i++) {
            if(lines_[i]->getLength() < 1.5f) {
                return true;
            }
        }

        return false;
    }

    int findBlockDeepestLineIndex() {
        int deepestLineIndex = 0;
        float deepestBlockValue = 0;
        for(int i = 0; i < 4; i++) {
            if(deepestBlockValue < lines_[i]->calcOrthDistOfWorldPointAbs(block_.worldPoint)) {
                deepestLineIndex = i;
                deepestBlockValue = lines_[i]->calcOrthDistOfWorldPointAbs(block_.worldPoint);
            }
        }
        return deepestLineIndex;
    }

    void recoverOriginUssSlot()
    {
        corners[0].worldCorner = lines_[0]->calcWorldPointUsingTauValue(originP0Tau);
        corners[1].worldCorner = lines_[0]->calcWorldPointUsingTauValue(originP1Tau);
        Eigen::Vector3d depthUV = 
            Fusion::turnUnitVectorFor90DegreeInAntiClockwiseInVehicleCoordinate(lines_[0]->getDefaultUnitVec());
        corners[2].worldCorner = corners[1].worldCorner +  depthUV*lines_[1]->getLength();
        corners[3].worldCorner = corners[0].worldCorner +  depthUV*lines_[3]->getLength();
    }

public:
    std::array<SlotCorner, 4U> corners;
    SlotCorner middlePoint;   //middle point will be calculated during slot init & after slot update
    std::array<EdgePoint, 2U> edges_;
    std::array<Fusion::CTrackLine::Ptr, 4U> lines_;
    //std::array<Fusion::CLineModel, 4U> lines_;
    double confidence_;
    BlockPoint block_;
    FixedSizeDeque<FusionSlot::Ptr> relativeSlots{20};
    std::vector<uint8_t> fsVehPointTypes;
    std::vector<uint8_t> fsOtherPointTypes;
    bool isOccupiedByLocker = false;
    bool isLockerConfirmed = false;

    bool isOccupiedByVeh = false;
    float lifeCounter = 1;
    int16_t updateActive = false;
    bool isConfirm = false;
    Status finalStatus = Status::DEFAULT;
    uint16_t id = 0xffff;
    FusionSlot::Ptr alignTrackSlot = nullptr;  //for detected slot
    uint8_t indexDiffWithAlignTrackSlotInAntiClockwise = 0U;

    Type type = Type::UNKNOWN;
    double width;
    double length;
    double angle;
    bool isReverse = false;

    AdjoinSlot ajoin;
    SlotLoc locForUssSlot = SlotLoc::UNKNOWN;
    
    AlignInfo alignInfo;

    FusionSlot::Ptr sourceSlot;
    ContributingSensor sensor = ContributingSensor::DEFAULT;
    bool isParkable = false;

    std::vector<matchedInfo> matchedList; //for tracked & detected slots
    matchedInfo lowestCostInfo; //for tracked & detected slots
    bool isShouldNotInited = false; //for detected Slot
    
    bool isEverBeforeFrontAxel = false;
    bool isEverAfterFrontAxel = false;

    bool isEverBeforeRearAxel = false;
    bool isEverAfterRearAxel = false;

    int openLineIndex = 4;

    bool isUpdatedThisCircle = false;

    float middleTau = 0.0f;
    float originP0Tau = 0.0f;
    float originP1Tau = 0.0f;

    Odometry initOdom;
    bool isMovingDistanceAfterInit = false;
};

class CSlotsList
{

public:
    void add(const FusionSlot::Ptr& newSlot)
    {
        for (size_t i = 1U; i < idPool_.size(); ++i) {
            if (idPool_[i] == false) {
                newSlot->id = i;
                idPool_[i] = true;
                break;
            }
        }
        slotsList_.push_back(newSlot);
    }

    void deleteSlot(const Odometry& poseOb)
    {
        for (auto iter = slotsList_.begin(); iter != slotsList_.end();) {
            const double distance = 
                (((*iter)->corners[0].worldCorner + (*iter)->corners[1].worldCorner) / 2.0 -
                poseOb.translation).norm();
            if ((*iter)->lifeCounter < -10 || (distance > 12.0) || (*iter)->getIsLengthTooShort()) {
                // if((*iter)->id == 5) {
                //     std::cout<<"slot 5 deleted"<<"\n";
                // }
                deleteSlot(iter);
            } else {
                ++iter;
            }
        }
    }

    void deleteSlot(std::list<FusionSlot::Ptr>::iterator& iter)
    {
        (*iter)->setPtrToNullptr();
        idPool_[(*iter)->id] = false;
        iter = slotsList_.erase(iter);
    }
    
    void replace(const FusionSlot::Ptr& oldSlot, const FusionSlot::Ptr& newSlot)
    {
        uint16_t id_ = oldSlot->id;
        FusionSlot::AlignInfo alignInfo_ = oldSlot->alignInfo;
        FusionSlot::ContributingSensor tempSensor = oldSlot->sensor;

        bool tempIsEverBeforeFrontAxel = oldSlot->isEverBeforeFrontAxel;
        bool tempIsEverAfterFrontAxel = oldSlot->isEverAfterFrontAxel;
        bool tempIsEverBeforeRearAxel = oldSlot->isEverBeforeRearAxel;
        bool tempIsEverAfterRearAxel = oldSlot->isEverAfterRearAxel;
        Odometry tempInitOdom = oldSlot->initOdom;
        bool tempIsMovingDistanceAfterInit = oldSlot->isMovingDistanceAfterInit;
        bool tempIsParkable = oldSlot->isParkable;

        *oldSlot = *newSlot;
        oldSlot->id = id_;
        oldSlot->alignInfo = alignInfo_;
        oldSlot->sensor = tempSensor;

        oldSlot->isEverBeforeFrontAxel = tempIsEverBeforeFrontAxel;
        oldSlot->isEverAfterFrontAxel = tempIsEverAfterFrontAxel;
        oldSlot->isEverBeforeRearAxel = tempIsEverBeforeRearAxel;
        oldSlot->isEverAfterRearAxel = tempIsEverAfterRearAxel;
        oldSlot->initOdom = tempInitOdom;
        oldSlot->isMovingDistanceAfterInit = tempIsMovingDistanceAfterInit;
        oldSlot->isParkable = tempIsParkable;
    }

    void reset()
    {
        for (auto iter = slotsList_.begin(); iter != slotsList_.end();) {
            deleteSlot(iter);
        }
        slotsList_.clear();

        std::array<bool, 0xffff> newIdPool_ = {false};
        idPool_ = newIdPool_;

        targetSlotId = 0;
        targetSlotLoc = FusionSlot::SlotLoc::UNKNOWN;
    }

    void resetEachCircleForPLDTracking()
    {
        for(const auto& tSlot : slotsList_) {
            tSlot->resetEachCircleForPLDTracking();
        }
    }

    std::list<FusionSlot::Ptr> slotsList_;
    std::array<bool, 0xffff> idPool_ = {false};

    uint32_t targetSlotId = 0U;
    FusionSlot::SlotLoc targetSlotLoc = FusionSlot::SlotLoc::UNKNOWN;
    
};

class CUssSlotsList : public CSlotsList
{
public:

    void Push_back(const FusionSlot::Ptr& newSlot, const uint8_t updateValue)
    {
        if(newSlot->locForUssSlot == FusionSlot::SlotLoc::LEFT) {
            if(updateValue == inValidLeftUpdateValue) {
                return;
            }
            else {
                inValidLeftUpdateValue = 16U;
            }
        }
        else {
            if(updateValue == inValidRightUpdateValue) {
                return;
            }
            else {
                inValidRightUpdateValue = 16U;
            }
        }

        for(const auto& tSlot : slotsList_) {

            if(Fusion::isPointInsideSlot(tSlot->lines_, newSlot->middlePoint.worldCorner)) {
                replace(tSlot, newSlot);
                if(newSlot->locForUssSlot == FusionSlot::SlotLoc::LEFT) {
                    leftUpdateValue = updateValue;
                    currentLeftSlot = tSlot;
                }
                else {
                    rightUpdateValue = updateValue;
                    currentRightSlot = tSlot;
                }
                return;
            }
        }
        
        this->add(newSlot);
        if(newSlot->locForUssSlot == FusionSlot::SlotLoc::LEFT) {
            leftUpdateValue = updateValue;
            currentLeftSlot = newSlot;
        }
        else {
            rightUpdateValue = updateValue;
            currentRightSlot = newSlot;
        }
    }
    void deleteSlot(std::list<FusionSlot::Ptr>::iterator& iter)
    {
        if((*iter) == currentLeftSlot) {
            currentLeftSlot = nullptr;
        }
        if((*iter) == currentRightSlot) {
            currentRightSlot = nullptr;
        }

        CSlotsList::deleteSlot(iter);
    }

    void deleteSlot(const Odometry& poseOb)
    {
        for(auto iter = slotsList_.begin(); iter != slotsList_.end();) {
            (*iter)->TransferToBase(poseOb);
            const double distance =
                (((*iter)->corners[0].baseCorner + (*iter)->corners[1].baseCorner) / 2.0).norm();
                //- poseOb.translation).norm();
            if(distance > 12.0) {
                CUssSlotsList::deleteSlot(iter);
            }
            else {
                ++iter;
            }
        }
    }

    void reset() {
        inValidLeftUpdateValue = (leftUpdateValue == 16)?inValidLeftUpdateValue:leftUpdateValue;
        inValidRightUpdateValue = (rightUpdateValue == 16)?inValidRightUpdateValue:rightUpdateValue;

        isLastFrameInParking = false;
        leftUpdateValue = 16U;
        currentLeftSlot = nullptr;
        rightUpdateValue = 16U;
        currentRightSlot = nullptr;
        CSlotsList::reset();
    }

    bool isLastFrameInParking = false;
    uint8_t leftUpdateValue = 16U;
    FusionSlot::Ptr currentLeftSlot = nullptr;
    uint8_t rightUpdateValue = 16U;
    FusionSlot::Ptr currentRightSlot = nullptr;

    uint8_t inValidLeftUpdateValue = 16U;
    uint8_t inValidRightUpdateValue = 16U;


};

}
}


#endif