#ifndef FUSION_CHASSIS_H_
#define FUSION_CHASSIS_H_

#include <cstdint>
#include <cmath>
#include "loc_msgs/msg/odom.hpp"
#include "vehicle_msgs/msg/ap_ibs20ms_pdu05.hpp"
#include "vehicle_msgs/msg/can_copy_cp2_ap20ms_struct.hpp"
#include "utils/time_stamp.h"

namespace Fusion {
using ChassisMsg = vehicle_msgs::msg::ApIbs20msPdu05;
using ChassisMsgPtr = ChassisMsg::SharedPtr;

using ChassisYawRateMsg = vehicle_msgs::msg::CanCopyCp2Ap20msStruct;
using ChassisYawRateMsgPtr = ChassisYawRateMsg::SharedPtr;

struct Chassis {
    Chassis() = default;
    Chassis(const loc_msgs::msg::ODOM::SharedPtr& msgPtr) :
        timestamp(msgPtr->header.stamp),
        wheelSpeedFrontLeft(msgPtr->front_left_wheel_spd),
        wheelSpeedFrontRight(msgPtr->front_right_wheel_spd),
        wheelSpeedRearLeft(msgPtr->rear_left_wheel_spd),
        wheelSpeedRearRight(msgPtr->rear_right_wheel_spd)
    {
        velocity = (wheelSpeedRearLeft + wheelSpeedRearRight) * 0.5;
    }

    Chassis(const vehicle_msgs::msg::ApIbs20msPdu05::SharedPtr& msgPtr) :
        timestamp(static_cast<uint64_t>(msgPtr->timestamp) * 1000000ULL),
        wheelSpeedFrontLeft(msgPtr->iwhlgndvelldrvn / 3.6),
        wheelSpeedFrontRight(msgPtr->iwhlgndvelrdrvn / 3.6),
        wheelSpeedRearLeft(msgPtr->iwhlgndvelldrvn / 3.6),
        wheelSpeedRearRight(msgPtr->iwhlgndvelrdrvn / 3.6)
    {
        wheelSpeedFrontLeft = msgPtr->ildrvnwhlrotldircn == 2U ? -wheelSpeedFrontLeft : wheelSpeedFrontLeft;
        wheelSpeedFrontRight = msgPtr->ildrvnwhlrotldircn == 2U ? -wheelSpeedFrontRight : wheelSpeedFrontRight;
        wheelSpeedRearLeft = msgPtr->ildrvnwhlrotldircn == 2U ? -wheelSpeedRearLeft : wheelSpeedRearLeft;
        wheelSpeedRearRight = msgPtr->ildrvnwhlrotldircn == 2U ? -wheelSpeedRearRight : wheelSpeedRearRight;
        velocity = (wheelSpeedRearLeft + wheelSpeedRearRight) * 0.5;
    }

    void SetYawRate(const ChassisYawRateMsgPtr& msgPtr)
    {
        this->isYawRateValid = true;
        double tmp = msgPtr->ibs20mspdu08.ivehdynyawrate;
        tmp = tmp > 128.0 ? (tmp - 256.0) : tmp;
        this->yawRate = (tmp * M_PI / 180.0F);
    }

    TimeStamp timestamp;
    double wheelSpeedFrontLeft = 0.0F;
    double wheelSpeedFrontRight = 0.0F;
    double wheelSpeedRearLeft = 0.0F;
    double wheelSpeedRearRight = 0.0F;
    double velocity = 0.0F;
    double gearAngle = 0.0F;

    bool isYawRateValid = false;
    double yawRate = 0.0F;

    using Ptr = std::shared_ptr<Chassis>;
};
}

#endif