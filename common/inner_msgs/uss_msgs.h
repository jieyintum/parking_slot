//
// Created by igs on 2022/12/13.
//

#ifndef USS_MSGS
#define USS_MSGS

#include "utils/pld_utils.h"
#include "msg_base.h"
#include "vector"
#include "eigen3/Eigen/Eigen"
#include "uss_msgs/msg/ap_uss.hpp"
#include "iostream"
#include "fusion_slots.h"
#include "../line_process/line_math_function.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

namespace Fusion
{

    using LeftSlotInfoMsg = uss_msgs::msg::APLeftSlotInformationTypeStruct;
    using LeftSlotInfoMsgPtr = LeftSlotInfoMsg::SharedPtr;

    using RightSlotInfoMsg = uss_msgs::msg::APRightSlotInformationTypeStruct;
    using RightSlotInfoMsgPtr = RightSlotInfoMsg::SharedPtr;

    enum class Loc : uint8_t
    {
        UNKNOWN = 0U,
        LEFT = 1U,
        RIGHT = 2U
    };

    enum class sideState : uint8_t
    {
        UNKNOWN = 0U,
        SINGLE_SIDE = 1U,
        DOUBLE_SIDE = 2U
    };

    struct UssSlot : SingleSlot
    {
        using Ptr = std::shared_ptr<UssSlot>;

        UssSlot() = default;

        ~UssSlot() = default;

        UssSlot(const LeftSlotInfoMsgPtr &msgPtr) : loc(Fusion::Loc::LEFT),
                                                    updateValue((uint8_t)msgPtr->left_slot_status_update),
                                                    isSlotInfoValid(msgPtr->left_slot_valid)
        {
            if ((Fusion::isZero(msgPtr->float_left_slot_location_bx) && Fusion::isZero(msgPtr->float_left_slot_location_by)) || (Fusion::isZero(msgPtr->float_left_slot_depth_px) && Fusion::isZero(msgPtr->float_left_slot_depth_py)) || (Fusion::isZero(msgPtr->float_left_slot_depth_qx) && Fusion::isZero(msgPtr->float_left_slot_depth_qy)) || (Fusion::isZero(msgPtr->float_left_slot_location_cx) && Fusion::isZero(msgPtr->float_left_slot_location_cy)))
            {
                isSlotInfoValid = false;
            }

            if (isSlotInfoValid)
            {
                type = std::uint8_t(msgPtr->left_slot_info_slot_type);

                float l_vehicleX_inMeter_f = (double)(msgPtr->left_slot_self_locationx / 1000.0f);
                float l_vehicleY_inMeter_f = (double)(msgPtr->left_slot_self_locationy / 1000.0f);
                float l_vehicleAngle_radian = ((double)msgPtr->left_slot_self_location_angle / 180.0f) * M_PI;

                p0[0] = (double)msgPtr->float_left_slot_location_bx / 1000.0f;
                p0[1] = (double)msgPtr->float_left_slot_location_by / 1000.0f;
                p0[2] = 0.0f;

                p0 = TransferToBase(
                    p0,
                    l_vehicleX_inMeter_f,
                    l_vehicleY_inMeter_f,
                    l_vehicleAngle_radian);

                p1[0] = (double)msgPtr->float_left_slot_location_cx / 1000.0f;
                p1[1] = (double)msgPtr->float_left_slot_location_cy / 1000.0f;
                p1[2] = 0.0f;
                p1 = TransferToBase(
                    p1,
                    l_vehicleX_inMeter_f,
                    l_vehicleY_inMeter_f,
                    l_vehicleAngle_radian);

                p2[0] = (double)msgPtr->float_left_slot_depth_qx / 1000.0f;
                p2[1] = (double)msgPtr->float_left_slot_depth_qy / 1000.0f;
                p2[2] = 0.0f;
                p2 = TransferToBase(
                    p2,
                    l_vehicleX_inMeter_f,
                    l_vehicleY_inMeter_f,
                    l_vehicleAngle_radian);

                p3[0] = (double)msgPtr->float_left_slot_depth_px / 1000.0f;
                p3[1] = (double)msgPtr->float_left_slot_depth_py / 1000.0f;
                p3[2] = 0.0f;
                p3 = TransferToBase(
                    p3,
                    l_vehicleX_inMeter_f,
                    l_vehicleY_inMeter_f,
                    l_vehicleAngle_radian);
            }
        }

        UssSlot(const RightSlotInfoMsgPtr &msgPtr) : loc(Fusion::Loc::RIGHT),
                                                     updateValue((uint8_t)msgPtr->right_slot_status_update),
                                                     isSlotInfoValid(msgPtr->right_slot_valid)
        {
            if ((Fusion::isZero(msgPtr->float_right_slot_location_bx) && Fusion::isZero(msgPtr->float_right_slot_location_by)) || (Fusion::isZero(msgPtr->float_right_slot_depth_px) && Fusion::isZero(msgPtr->float_right_slot_depth_py)) || (Fusion::isZero(msgPtr->float_right_slot_depth_qx) && Fusion::isZero(msgPtr->float_right_slot_depth_qy)) || (Fusion::isZero(msgPtr->float_right_slot_location_cx) && Fusion::isZero(msgPtr->float_right_slot_location_cy)))
            {
                isSlotInfoValid = false;
            }

            if (isSlotInfoValid)
            {
                type = std::uint8_t(msgPtr->right_slot_info_slot_type);

                float l_vehicleX_inMeter_f = (double)(msgPtr->right_slot_self_locationx / 1000.0f);
                float l_vehicleY_inMeter_f = (double)(msgPtr->right_slot_self_locationy / 1000.0f);
                float l_vehicleAngle_radian = ((double)msgPtr->right_slot_self_location_angle / 180.0f) * M_PI;

                p0[0] = (double)msgPtr->float_right_slot_location_cx / 1000.0f;
                p0[1] = (double)msgPtr->float_right_slot_location_cy / 1000.0f;
                p0[2] = 0.0f;

                p0 = TransferToBase(
                    p0,
                    l_vehicleX_inMeter_f,
                    l_vehicleY_inMeter_f,
                    l_vehicleAngle_radian);

                p1[0] = (double)msgPtr->float_right_slot_location_bx / 1000.0f;
                p1[1] = (double)msgPtr->float_right_slot_location_by / 1000.0f;
                p1[2] = 0.0f;

                p1 = TransferToBase(
                    p1,
                    l_vehicleX_inMeter_f,
                    l_vehicleY_inMeter_f,
                    l_vehicleAngle_radian);

                p2[0] = (double)msgPtr->float_right_slot_depth_px / 1000.0f;
                p2[1] = (double)msgPtr->float_right_slot_depth_py / 1000.0f;
                p2[2] = 0.0f;

                p2 = TransferToBase(
                    p2,
                    l_vehicleX_inMeter_f,
                    l_vehicleY_inMeter_f,
                    l_vehicleAngle_radian);

                p3[0] = (double)msgPtr->float_right_slot_depth_qx / 1000.0f;
                p3[1] = (double)msgPtr->float_right_slot_depth_qy / 1000.0f;
                p3[2] = 0.0f;

                p3 = TransferToBase(
                    p3,
                    l_vehicleX_inMeter_f,
                    l_vehicleY_inMeter_f,
                    l_vehicleAngle_radian);
            }
        }

        Eigen::Vector3d TransferToBase(
            const Eigen::Vector3d inputPoint,
            const float &f_vehicleX_f,
            const float &f_vehicleY_f,
            const float &f_vehicleAng_f)
        {
            Eigen::Vector3d deltaVec;
            deltaVec[0] = inputPoint[0] - f_vehicleX_f;
            deltaVec[1] = inputPoint[1] - f_vehicleY_f;
            deltaVec[2] = 0.0f;

            float l_distance_f = Fusion::calcLength(deltaVec);
            Eigen::Vector3d l_unitVector_t = Fusion::getUnitVector(deltaVec);

            float l_relativeAngle_radian = std::asin(l_unitVector_t[1]);
            if (l_relativeAngle_radian < 0.0f)
            {
                if (l_unitVector_t[0] < 0.0f)
                {
                    l_relativeAngle_radian = -(M_PI + l_relativeAngle_radian);
                }
                else
                {
                    // do nothing
                }
            }
            else
            {
                if (l_unitVector_t[0] < 0.0f)
                {
                    l_relativeAngle_radian = M_PI - l_relativeAngle_radian;
                }
                else
                {
                    // do nothing
                }
            }

            float l_angleInVCoord_radian = l_relativeAngle_radian - f_vehicleAng_f;

            return Eigen::Vector3d(
                std::cos(l_angleInVCoord_radian) * l_distance_f,
                std::sin(l_angleInVCoord_radian) * l_distance_f,
                0.0f);
        }

        void ToVisuMarker(visualization_msgs::msg::Marker &returnMarker)
        {
            returnMarker.header.frame_id = "base_link";
            returnMarker.ns = "percept";
            returnMarker.type = 4;
            returnMarker.action = 0;

            returnMarker.scale.x = 0.05;
            returnMarker.scale.y = 0.05;
            returnMarker.scale.z = 0.05;

            std_msgs::msg::ColorRGBA color;
            color.r = 0.5;
            color.g = 1;
            color.b = 0;
            color.a = 1;
            returnMarker.color = color;
            returnMarker.lifetime = rclcpp::Duration::from_seconds(0.2);

            geometry_msgs::msg::Point point;

            point.x = this->p0[0];
            point.y = this->p0[1];
            point.z = 0.0;
            returnMarker.points.push_back(std::move(point));
            returnMarker.colors.push_back(std::move(color));

            point.x = this->p1[0];
            point.y = this->p1[1];
            point.z = 0.0;
            returnMarker.points.push_back(std::move(point));
            returnMarker.colors.push_back(std::move(color));

            point.x = this->p2[0];
            point.y = this->p2[1];
            point.z = 0.0;
            returnMarker.points.push_back(std::move(point));
            returnMarker.colors.push_back(std::move(color));

            point.x = this->p3[0];
            point.y = this->p3[1];
            point.z = 0.0;
            returnMarker.points.push_back(std::move(point));
            returnMarker.colors.push_back(std::move(color));
        }

        Fusion::Loc loc = Fusion::Loc::UNKNOWN;
        uint8_t updateValue = 0U;
        bool isSlotInfoValid = false;
    };

    using uss_msg = uss_msgs::msg::APUss;
    using uss_MsgPtr = uss_msg::SharedPtr;

    struct UssMsg : MsgBase
    {
        using Ptr = std::shared_ptr<UssMsg>;

        UssMsg() = default;

        ~UssMsg() = default;

        UssMsg(const uss_MsgPtr &msgPtr) : MsgBase(msgPtr->header.stamp),
                                           frameId(msgPtr->header.frame_id)

        {
            leftSlot = std::make_shared<UssSlot>(std::make_shared<LeftSlotInfoMsg>(msgPtr->left_slot_info));
            rightSlot = std::make_shared<UssSlot>(std::make_shared<RightSlotInfoMsg>(msgPtr->right_slot_info));
        }
#if DEBUG_MODE
        visualization_msgs::msg::MarkerArray ToMarkerArray(const builtin_interfaces::msg::Time &timeStamp)
        {
            visualization_msgs::msg::MarkerArray markers;
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = timeStamp;
            marker.id = 0U;

            if (this->leftSlot->isSlotInfoValid)
            {
                this->leftSlot->ToVisuMarker(marker);
                markers.markers.emplace_back(marker);
                marker.id++;
            }

            if (this->rightSlot->isSlotInfoValid)
            {
                this->rightSlot->ToVisuMarker(marker);
                markers.markers.emplace_back(marker);
            }
            return markers;
        }
#endif

        string frameId;

        UssSlot::Ptr leftSlot;
        UssSlot::Ptr rightSlot;
    };

}
#endif // USS_MSGS
