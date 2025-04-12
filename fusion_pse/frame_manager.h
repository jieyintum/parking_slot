#ifndef FUSION_PSE_FRAMEMANAGER_H_
#define FUSION_PSE_FRAMEMANAGER_H_

#include "utils/fixed_size_deque.h"
#include "inner_msgs/percept_slot.h"
#include "inner_msgs/avpe.h"

#include "inner_msgs/uss_msgs.h"
#include "inner_msgs/diagnostic.h"
#include "inner_msgs/fusion_slots.h"
#include "inner_msgs/freespace.h"

#include "inner_msgs/avpe_segmentation.h"

#include "msg_buffer/odom_buffer.h"
#include "pld_fusion/pld_frame.h"
#include "arrow_fusion/arrow_frame.h"
#include "zebra_fusion/zebra_frame.h"

#include "speed_bump_fusion/speed_bump_frame.h"

#include "lane_fusion/lane_frame.h"

namespace Fusion
{
    namespace PSE
    {
        class FrameManager
        {
        public:
            FrameManager() = default;
            ~FrameManager() = default;

            void Push(const FSStruct::Ptr &fsStructPtr)
            {
                fsStructs_.Push(fsStructPtr);
            }

            void Push(const FusionSlots::Ptr &pldTrackingPtr)
            {
                trackingSlots_.Push(pldTrackingPtr);
            }

            void Push(const PerceptSlot::Ptr &pldPtr)
            {
                plds_.Push(pldPtr);
            }

            void Push(const UssMsg::Ptr &ussPtr)
            {
                ussS_.Push(ussPtr);
            }

            void Push(const Avpe::Ptr &avpePtr)
            {
                avpes_.Push(avpePtr);
            }

            void Push(const Diagnostic::Ptr &drDiagnosticPtr)
            {
                drDiagnostic_.Push(drDiagnosticPtr);
            }

            void Push(const StaticElements::Ptr &wallPillrsPtr)
            {
                wallPillars_.Push(wallPillrsPtr);
            }

            void Push(const AvpeSegArray::Ptr &avpeSegPtr)
            {
                segs_.Push(avpeSegPtr);
            }

            bool MakeFrame(PldFrame &pldFrame)
            {

                // OdomBuffer::GetInstance().Back()->GetTime().Sec() is always behind pldFrame.uss_->timestamp.Sec()

                if (ussS_.GetNewest(pldFrame.uss_))
                {
                }
                else
                {
                    pldFrame.uss_ = nullptr;
                }

                Odometry::Ptr odomPtr = nullptr;
                if (!OdomBuffer::GetInstance().Empty())
                {
                    odomPtr = OdomBuffer::GetInstance().Back();
                    pldFrame.poseOb = *odomPtr;
                }
                else
                {
                    pldFrame.uss_ = nullptr;
                }

                if ((!OdomBuffer::GetInstance().Empty()) && (fsStructs_.GetNewest(pldFrame.fs_)))
                {
                    Odometry::Ptr fsOdomPtr = OdomBuffer::GetInstance().GetInterpolation(pldFrame.fs_->timestamp);
                    if (fsOdomPtr == nullptr)
                    {
                        pldFrame.fs_ = nullptr;
                    }
                }
                else
                {
                    pldFrame.fs_ = nullptr;
                }

                // return (pldFrame.uss_ != nullptr);

                if ((!OdomBuffer::GetInstance().Empty()) && (plds_.GetNewest(pldFrame.pld_)))
                {
                    pldFrame.stamp = pldFrame.pld_->timestamp;
                    odomPtr = OdomBuffer::GetInstance().GetInterpolation(pldFrame.stamp);
                    if (odomPtr != nullptr)
                    {

                        lastValidPldTime = rclcpp::Clock().now(); // pldFrame.stamp;
                        pldFrame.poseOb = *odomPtr;
                    }
                    else
                    {
                        pldFrame.pld_ = nullptr;
                    }

                    if (!avpes_.Empty())
                    {
                        const auto lastAvpe = avpes_.Back();
                        pldFrame.vsFs_ = std::make_shared<VisionFreespace>(lastAvpe->freespace, lastAvpe->timestamp);
                        pldFrame.vsObject_ = std::make_shared<visionObjVector>(lastAvpe, lastAvpe->timestamp);
                        // std::cout<<"vs obj count: "<<pldFrame.vsObject_->vsObjVector.size()<<"\n";
                        //  for(const auto& obj :  pldFrame.vsObject_->vsObjVector) {
                        //      std::cout<<" rotatedBbox num: "<<obj->worldPoints.size();
                        //  }
                        //  std::cout<<"\n";
                    }
                }
                else
                {
                    pldFrame.pld_ = nullptr;
                }

                bool returnValue(false);

                TimeStamp currentTime;
                currentTime = rclcpp::Clock().now();
                if ((currentTime.Sec() - lastValidPldTime.Sec()) > 1.5)
                {
                    // std::cout<<"pure uss mode"<<"\n";
                    returnValue = (pldFrame.uss_ != nullptr);
                }
                else
                {
                    // std::cout<<"uss&pld mode"<<"\n";
                    returnValue = (pldFrame.pld_ != nullptr);
                }

                return returnValue;
            }

            bool MakeFrame(AvpeArrowFrame &avpeArrowFrame)
            {
                if (segs_.empty())
                {
                    return false;
                }
                AvpeSegArray::Ptr lastAvpePtr = segs_.Back();
                if (lastAvpePtr == nullptr)
                {
                    return false;
                }
                avpeArrowFrame.stamp = lastAvpePtr->timestamp;
                Odometry::Ptr odomPtr = OdomBuffer::GetInstance().GetInterpolation(avpeArrowFrame.stamp);
                if (odomPtr != nullptr)
                {
                    avpeArrowFrame.poseOb = *odomPtr;
                }
                avpeArrowFrame.detectArrow_ = std::make_shared<FusionArrow>();
                auto segmetations = lastAvpePtr->segmentations;
                if (segmetations.find(AvpeSegmentation::Type::ARROW) == segmetations.end())
                {
                    std::cout << "Cannot find arrow segmetaion!" << std::endl;
                    return true;
                }
                int arrowType = segmetations[AvpeSegmentation::Type::ARROW].type;
                auto arrowSegPoints = segmetations[AvpeSegmentation::Type::ARROW].region;
                for (const auto &avpPoint : arrowSegPoints)
                {
                    avpeArrowFrame.detectArrow_->points.emplace_back(avpPoint.point.x(), avpPoint.point.y(), 0.0);
                    avpeArrowFrame.detectArrow_->pixleTypes.push_back(arrowType);
                }

                if (odomPtr != nullptr)
                {
                    avpeArrowFrame.detectArrow_->TransferToWorld(avpeArrowFrame.poseOb);
                }

                return odomPtr != nullptr;
            }

            bool MakeFrame(AvpeZebraFrame &avpeZebraFrame)
            {
                if (segs_.empty())
                {
                    return false;
                }
                AvpeSegArray::Ptr lastAvpePtr = segs_.Back();
                if (lastAvpePtr == nullptr)
                {
                    return false;
                }
                avpeZebraFrame.detectZebra_ = std::make_shared<FusionZebra>();
                auto segmetations = lastAvpePtr->segmentations;
                if (segmetations.find(AvpeSegmentation::Type::ZEBRA) == segmetations.end())
                {
                    std::cout << "Cannot find zebra segmetaion!" << std::endl;
                    return false;
                }
                auto zebraSegPoints = segmetations[AvpeSegmentation::Type::ZEBRA].region;
                for (const auto &avpPoint : zebraSegPoints)
                {
                    avpeZebraFrame.detectZebra_->points.emplace_back(avpPoint.point.x(), avpPoint.point.y(), 0.0);
                }
                avpeZebraFrame.stamp = lastAvpePtr->timestamp;
                Odometry::Ptr odomPtr = OdomBuffer::GetInstance().GetInterpolation(avpeZebraFrame.stamp);
                if (odomPtr != nullptr)
                {
                    avpeZebraFrame.poseOb = *odomPtr;
                    avpeZebraFrame.detectZebra_->TransferToWorld(avpeZebraFrame.poseOb);
                }
                return odomPtr != nullptr;
            }

            bool MakeFrame(AvpeSpeedBumpFrame &avpeSpeedBumpFrame)
            {
                if (avpes_.empty())
                {
                    return false;
                }
                Avpe::Ptr lastAvpePtr = avpes_.Back();
                if (lastAvpePtr == nullptr)
                {
                    return false;
                }
                avpeSpeedBumpFrame.stamp = lastAvpePtr->timestamp;
                Odometry::Ptr odomPtr = OdomBuffer::GetInstance().GetInterpolation(avpeSpeedBumpFrame.stamp);
                if (odomPtr != nullptr)
                {
                    avpeSpeedBumpFrame.poseOb = *odomPtr;
                    avpeSpeedBumpFrame.is_ego_static_ = std::abs(avpeSpeedBumpFrame.poseOb.linearVelocity.x()) < 0.1 ? true : false;

                    for (auto &ptr : lastAvpePtr->objects)
                    {
                        if (ptr.type != 16 || ptr.rotatedBbox.size() != 4)
                            continue;

                        SpeedBumpTrack::Ptr obj_ptr = std::make_shared<SpeedBumpTrack>();
                        obj_ptr->detect_id = ptr.id;
                        obj_ptr->classification = StaticClassification::STATIC_SPEED_BUMP;
                        obj_ptr->number_of_vertices = ptr.rotatedBbox.size();
                        obj_ptr->vertices.resize(obj_ptr->number_of_vertices);
                        for (int i = 0; i < obj_ptr->number_of_vertices; ++i)
                        {
                            obj_ptr->vertices[i].x() = ptr.rotatedBbox[i].point.x();
                            obj_ptr->vertices[i].y() = ptr.rotatedBbox[i].point.y();
                            obj_ptr->vertices[i].z() = 0.0;
                        }

                        obj_ptr->GetCenterPoints();
                        obj_ptr->TransferToWorld(avpeSpeedBumpFrame.poseOb);
                        if (obj_ptr->GetWorldRectWidth() > 1.5)
                            continue;

                        avpeSpeedBumpFrame.detect_speed_bumps_.emplace_back(obj_ptr);
                    }
                }
                else
                {
                    std::cout << "get odometry failed!" << std::endl;
                }
                return odomPtr != nullptr;
            }

            Avpe::Ptr GetLastAvpePtr()
            {
                if (avpes_.empty())
                {
                    return nullptr;
                }
                return avpes_.Back();
            }

            bool MakeFrame(AvpeLaneFrame &avpeLaneFrame)
            {
                if (segs_.empty())
                {
                    return false;
                }
                avpeLaneFrame.lane_ = segs_.Back();
                if (avpeLaneFrame.lane_ == nullptr)
                {
                    return false;
                }
                const auto segmentations = avpeLaneFrame.lane_->segmentations;
                if (segmentations.find(AvpeSegmentation::Type::LANE) == segmentations.end())
                {
                    return false;
                }
                avpeLaneFrame.stamp = avpeLaneFrame.lane_->timestamp;
                Odometry::Ptr odomPtr = OdomBuffer::GetInstance().GetInterpolation(avpeLaneFrame.stamp);
                if (odomPtr != nullptr)
                {
                    avpeLaneFrame.poseOb = *odomPtr;
                }
                return odomPtr != nullptr;
            }

            FixedSizeDeque<Diagnostic::Ptr> drDiagnostic_{10};
            FixedSizeDeque<FusionSlots::Ptr> trackingSlots_{10};
            FixedSizeDeque<FSStruct::Ptr> fsStructs_{10};

            StaticElements::Ptr GetNewestWallPillars()
            {
                StaticElements::Ptr output = nullptr;
                wallPillars_.GetNewest(output);
                return output;
            }

            AvpeSegArray::Ptr GetLastSegPtr()
            {
                if (segs_.empty())
                {
                    return nullptr;
                }
                return segs_.Back();
            }

        private:
            FixedSizeDeque<PerceptSlot::Ptr> plds_{10U};

            FixedSizeDeque<StaticElements::Ptr> wallPillars_{10U};
            FixedSizeDeque<AvpeSegArray::Ptr> segs_{10};

            TimeStamp lastValidPldTime;

            FixedSizeDeque<UssMsg::Ptr> ussS_{20U};
            FixedSizeDeque<Avpe::Ptr> avpes_{10U};
        };

    }
}
#endif