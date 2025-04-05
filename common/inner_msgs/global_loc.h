
#ifndef FUSION_GLOBAL_LOC_H_
#define FUSION_GLOBAL_LOC_H_

#include <memory>
#include <Eigen/Eigen>
#include "inner_msg/msg_base.h"
#include "utils/constant.h"
#include "loc_msgs/msg/global_loc.hpp"
namespace Fusion {
using GLocMsg = loc_msgs::msg::GlobalLoc;
using GLocMsgPtr = GLocMsg::SharedPtr;
struct GlobalLoc : MsgBase {
    using Ptr = std::shared_ptr<GlobalLoc>;
    std::uint8_t status;
    std::uint16_t source;
    std::uint8_t faultFlag;
    Eigen::Vector3d lla;
    Eigen::Vector3d vel;
    Eigen::Vector3d eular;
    Eigen::Vector3d llaDev;
    Eigen::Vector3d velDev;
    Eigen::Vector3d attDev;
    
    Eigen::Vector3d mapCenter;
    uint64_t roadId;
    GlobalLoc() = default;
    GlobalLoc(const GLocMsgPtr msg)
        : MsgBase(msg->header.stamp), status(msg->status), source(msg->source),
          faultFlag(msg->falut_flag), lla(msg->lla.latitude, msg->lla.longitude, msg->lla.altitude),
          vel(msg->velocity.x, msg->velocity.y, msg->velocity.z),
          eular(msg->attitude.roll, msg->attitude.pitch, msg->attitude.yaw),
          llaDev(msg->position_accuracy.x, msg->position_accuracy.y, msg->position_accuracy.z),
          velDev(msg->velocity_accuracy.x, msg->velocity_accuracy.y, msg->velocity_accuracy.z),
          attDev(msg->attitude_accuracy.roll, msg->attitude_accuracy.pitch, msg->attitude_accuracy.yaw),
          mapCenter(msg->map_center.latitude, msg->map_center.longitude, msg->map_center.altitude), roadId(msg->road_id) {}

    void ToGLocMsg(GLocMsg& msg)
    {
        msg.header.stamp = timestamp.ToMsg();
        msg.header.frame_id = "lla";

        msg.lla.latitude = lla.x() * Constant::RAD2DEG;
        msg.lla.longitude = lla.y() * Constant::RAD2DEG;
        msg.lla.altitude = lla.z();

        msg.attitude.roll = eular.x();
        msg.attitude.pitch = eular.y();
        msg.attitude.yaw = eular.z() - M_PI / 2.0;

        msg.velocity.x = vel.y();
        msg.velocity.y = -vel.x();
        msg.velocity.z = vel.z();
    }
};
}

#endif