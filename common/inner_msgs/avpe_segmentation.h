//
// Created by igs on 2023/3/15.
//

#ifndef PARKING_SE_FUSION_AVPE_SEGMENTATION_H
#define PARKING_SE_FUSION_AVPE_SEGMENTATION_H

#include "msg_base.h"
#include "vision_msgs/msg/avpe_segmentation_array.hpp"

namespace Fusion
{
    struct SegPoint
    {
        SegPoint() = default;

        ~SegPoint() = default;

        SegPoint(const double pointXIn,
                 const double pointYIn,
                 const int typeIn,
                 const float confIn,
                 const int groundingIn, const bool isPixelCoord = false) : point(pointXIn, pointYIn),
                                                                           conf(confIn),
                                                                           type(typeIn),
                                                                           grounding(groundingIn)
        {
            if (isPixelCoord)
            {
                TransforToVehicleCoord();
            }
        }

        void TransforToVehicleCoord()
        {
            PldUtils::Point2f p(static_cast<float>(point.y()), static_cast<float>(point.x()));
            PointCoordinateTransfer(p);
            point.x() = static_cast<double>(p.x);
            point.y() = static_cast<double>(p.y);
        }

        Eigen::Vector2d point;
        float conf;
        int type;
        int grounding;
    };

    struct AvpeSegmentation
    {
        using Ptr = std::shared_ptr<AvpeSegmentation>;
        enum class Type
        {
            FS = 0,
            LANE = 1,
            ARROW = 2,
            ZEBRA = 3,
            SLOT = 4
        };

        AvpeSegmentation() = default;

        AvpeSegmentation(const vision_msgs::msg::AvpeSegmentation &msg)
        {
            Insert(msg);
        }

        void Insert(const vision_msgs::msg::AvpeSegmentation &msg)
        {
            task = msg.task;
            type = msg.type;
            subtype = msg.subtype;
            conf = msg.conf;

            region.reserve(region.size() + msg.region.size());
            for (const auto &point : msg.region)
            {
                region.emplace_back(point.x, point.y, point.type, point.conf, point.grounding, true);
            }
        }

        ~AvpeSegmentation() = default;

        std::vector<SegPoint> region;
        int task = 0;
        int type = 0;
        int subtype = 0;
        float conf = 0;
    };

    using AvpeSegMsg = vision_msgs::msg::AvpeSegmentation;
    using AvpeSegArrayMsg = vision_msgs::msg::AvpeSegmentationArray;
    using AvpeSegArrayMsgPtr = AvpeSegArrayMsg::SharedPtr;

    struct AvpeSegArray : MsgBase
    {
        using Ptr = std::shared_ptr<AvpeSegArray>;

        AvpeSegArray() = default;

        ~AvpeSegArray() = default;

        AvpeSegArray(const AvpeSegArrayMsgPtr &msgPtr) : MsgBase(msgPtr->header.stamp),
                                                         frameId(msgPtr->frameid),
                                                         cameraId(msgPtr->cameraid),
                                                         width(msgPtr->width),
                                                         height(msgPtr->height),
                                                         segNum(msgPtr->seg_num)
        {
            if (!msgPtr->segmentation.empty())
            {
                for (const auto &seg : msgPtr->segmentation)
                {
                    if (seg.task == 2U)
                    {
                        segmentations[AvpeSegmentation::Type::LANE].Insert(seg);
                    }
                    else if (seg.task == 3U)
                    {
                        if (seg.type < 14U && seg.type > 0U)
                        {
                            segmentations[AvpeSegmentation::Type::ARROW].Insert(seg);
                        }
                        else if (seg.type == 14U)
                        {
                            segmentations[AvpeSegmentation::Type::ZEBRA].Insert(seg);
                        }
                        else if (seg.type == 15U)
                        {
                            segmentations[AvpeSegmentation::Type::SLOT].Insert(seg);
                        }
                        else
                        {
                            /* do nothing*/
                        }
                    }
                    else
                    {
                        /* do nothing for now */
                    }
                }
            }
        }

        int frameId = 0;
        int cameraId = 0;
        int width = 0;
        int height = 0;
        int segNum = 0;
        std::map<AvpeSegmentation::Type, AvpeSegmentation> segmentations;
    };
}

#endif // PARKING_SE_FUSION_AVPE_SEGMENTATION_H