#ifndef FUSION_GNSS_H_
#define FUSION_GNSS_H_

#include <cstdint>
#include <Eigen/Eigen>

#include "loc_msgs/msg/gnss.hpp"
#include "utils/time_stamp.h"
#include "utils/constant.h"
#include "utils/matrix_utils.h"

namespace Fusion {
using GnssMsg = loc_msgs::msg::GNSS;
using GnssMsgPtr = GnssMsg::SharedPtr;

struct Gnss {
enum RTKState : std::uint8_t {
    INVALID = 0U,
    SPP = 1U,
    DGNSS = 2U,
    FIX = 4U,
    FLOAT = 5U
};
public:
    Gnss()
    {
        accuracy.resize(7);
    }
    ~Gnss() = default;

    Gnss(const loc_msgs::msg::GNSS::SharedPtr& msgPtr) : timestamp(msgPtr->header.stamp),
        latitude(msgPtr->lla.latitude * Constant::DEG2RAD), longitude(msgPtr->lla.longitude * Constant::DEG2RAD), altitude(msgPtr->lla.altitude),
        velEast(-msgPtr->velocity.y), velNorth(msgPtr->velocity.x), velUp(msgPtr->velocity.z),
        heading(msgPtr->heading),
        status(static_cast<RTKState>(msgPtr->status)),
        hdop(msgPtr->hdop), vdop(msgPtr->vdop),
        usedSatNum(msgPtr->satellite_used_num), highQualitySatNum(msgPtr->satellite_highquality_num)
    {
        accuracy.reserve(6);
        accuracy.push_back(msgPtr->position_accuracy.x);
        accuracy.push_back(msgPtr->position_accuracy.y);
        accuracy.push_back(msgPtr->position_accuracy.z);
        accuracy.push_back(msgPtr->velocity_accuracy.y);
        accuracy.push_back(msgPtr->velocity_accuracy.x);
        accuracy.push_back(msgPtr->velocity_accuracy.z);
    }
public:
    TimeStamp timestamp;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double velEast = 0.0;
    double velNorth = 0.0;
    double velUp = 0.0;
    double heading = 0.0;
    std::vector<double> accuracy; // plat, plon, palt, pvelE, pvelN, pvelU, (phead) 

    RTKState status = INVALID;
    double hdop;
    double vdop;
    std::uint8_t usedSatNum = 0U;
    std::uint8_t highQualitySatNum = 0U;

    using Ptr = std::shared_ptr<Gnss>;
};
}


#endif