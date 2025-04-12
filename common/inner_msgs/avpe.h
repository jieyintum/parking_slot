//
// Created by igs on 2022/12/13.
//

#ifndef LANE_AVPE_H
#define LANE_AVPE_H

#include "utils/pld_utils.h"
#include "msg_base.h"
#include "vector"
#include "Eigen/Eigen"
#include "vision_msgs/msg/avpe.hpp"
#include "line_object/line_base.h"
#include "iostream"

namespace Fusion
{

    struct AvpePoint
    {
        AvpePoint() = default;

        ~AvpePoint() = default;

        AvpePoint(const double pointXIn,
                  const double pointYIn,
                  const int typeIn,
                  const float confIn,
                  const int groundingIn, const bool isPixelCoord = false) : point(pointXIn, pointYIn),
                                                                            conf(confIn), type(typeIn),
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

    struct AvpePolygon
    {
        AvpePolygon() = default;

        ~AvpePolygon() = default;

        AvpePolygon(const vision_msgs::msg::AvpePolygon &msg) : id(msg.id),
                                                                type(msg.type),
                                                                conf(msg.conf),
                                                                trackedAge(msg.tracked_age),
                                                                occlusion(msg.occlusion),
                                                                truncation(msg.truncation),
                                                                direction(msg.direction)
        {
            for (const auto &point : msg.seg_points)
            {
                segPoints.emplace_back(point.x, point.y, point.type, point.conf, point.grounding);
            }
        }

        std::vector<AvpePoint> segPoints;
        int32_t id;
        int type;
        float conf;
        int32_t trackedAge;
        int occlusion;
        int truncation;
        int direction;
    };

    struct AvpeFreespace
    {
        using Ptr = std::shared_ptr<AvpeFreespace>;
        AvpeFreespace() = default;

        AvpeFreespace(const vision_msgs::msg::AvpeFreespace &msg) : numOfPoints(msg.number_of_points),
                                                                    conf(msg.conf)
        {
            points.reserve(numOfPoints);
            for (const auto &point : msg.points)
            {
                points.emplace_back(point.x, point.y, point.type, point.conf, point.grounding);
            }
        }

        ~AvpeFreespace() = default;

        int numOfPoints = 0;
        std::vector<AvpePoint> points;
        float conf = 0;
    };

    struct AvpeLane : LineBase
    {
        enum class LinePositionType
        {
            UNKNOW = 0,
            FIRST_ON_THE_LEFT = 1,
            SECOND_ON_THE_LEFT = 2,
            THIRD_ON_THE_LEFT = 3,
            FORTH_ON_THE_LEFT = 4,
            FIFTH_ON_THE_LEFT = 5,
            SIXTH_ON_THE_LEFT = 6,
            SEVENTH_ON_THE_LEFT = 7,
            FIRST_ON_THE_RIGHT = 8,
            SECOND_ON_THE_RIGHT = 9,
            THIRD_ON_THE_RIGHT = 10,
            FORTH_ON_THE_RIGHT = 11,
            FIFTH_ON_THE_RIGHT = 12,
            SIXTH_ON_THE_RIGHT = 13,
            SEVENTH_ON_THE_RIGHT = 14,
            OTHER = 15
        };

        AvpeLane(const vision_msgs::msg::AvpeLane &msg)
            : id(msg.id),
              pointsNum(msg.number_of_points),
              position(static_cast<LinePositionType>(msg.position)),
              type(msg.type),
              color(msg.color)
        {
            linePoints.reserve(pointsNum);
            for (const auto &point : msg.line_points)
            {
                linePoints.emplace_back(point.x, point.y, point.type, point.conf, point.grounding);
            }
        }

        ~AvpeLane() = default;

        uint32_t id = 0;
        int pointsNum = 0;
        std::vector<AvpePoint> linePoints;
        LinePositionType position;
        int type = 0;
        int color = 0;
    };

    struct AvpeObjects
    {
        using Ptr = std::shared_ptr<AvpeObjects>;
        AvpeObjects() = default;

        AvpeObjects(const vision_msgs::msg::AvpeObjects &msg) : position(msg.position),
                                                                type(msg.type),
                                                                color(msg.color),
                                                                conf(msg.conf),
                                                                trackedAge(msg.tracked_age),
                                                                occlusion(msg.occlusion),
                                                                truncation(msg.truncation),
                                                                direction(msg.direction),
                                                                id(msg.id)
        {
            for (const auto &point : msg.orderedpoints)
            {
                orderedPoints.emplace_back(point.x, point.y, point.type, point.conf, point.grounding);
            }
            for (const auto &point : msg.cornerpoints)
            {
                cornerPoints.emplace_back(point.x, point.y, point.type, point.conf, point.grounding);
            }
            for (const auto &point : msg.rotatedbbox)
            {
                rotatedBbox.emplace_back(point.x, point.y, point.type, point.conf, point.grounding);
            }
            for (const auto &polygon : msg.polygons)
            {
                polygons.emplace_back(polygon);
            }
        }

        ~AvpeObjects() = default;

        int position = 0;
        int type = 0;
        int color = 0;
        float conf = 0;
        int32_t trackedAge = 0;
        int occlusion = 0;
        int truncation = 0;
        int direction = 0;
        int32_t id = 0;
        std::vector<AvpePoint> orderedPoints;
        std::vector<AvpePoint> cornerPoints;
        std::vector<AvpePoint> rotatedBbox;
        std::vector<AvpePolygon> polygons;
    };

    using AvpeMsg = vision_msgs::msg::Avpe;
    using AvpeMsgPtr = AvpeMsg::SharedPtr;

    struct Avpe : MsgBase
    {
        using Ptr = std::shared_ptr<Avpe>;

        Avpe() = default;

        ~Avpe() = default;

        Avpe(const AvpeMsgPtr &msgPtr) : MsgBase(msgPtr->header.stamp),
                                         frameId(msgPtr->frameid),
                                         cameraId(msgPtr->cameraid),
                                         width(msgPtr->width),
                                         height(msgPtr->height),
                                         laneNum(msgPtr->lane_num),
                                         lmSegNum(msgPtr->lm_seg_num)
        {
            // freespace
            freespace = std::make_shared<AvpeFreespace>(msgPtr->freespace);

            // lanes
            if (!msgPtr->lanes.empty())
            {
                for (const auto &lane : msgPtr->lanes)
                {
                    lanes.emplace_back(lane);
                }
            }

            // objects
            if (!msgPtr->objects.empty())
            {
                for (const auto &object : msgPtr->objects)
                {
                    objects.emplace_back(object);
                }
            }
        }

        int32_t frameId = 0;
        int32_t cameraId = 0;
        int32_t width = 0;
        int32_t height = 0;
        uint16_t laneNum = 0;
        uint16_t lmSegNum = 0;

        AvpeFreespace::Ptr freespace;
        std::vector<AvpeObjects> objects;
        std::vector<AvpeLane> lanes;
    };
}
#endif // LANE_AVPE_H
