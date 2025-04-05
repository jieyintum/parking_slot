#ifndef FUSION_GNSS_H_
#define FUSION_GNSS_H_

#include <cstdint>
#include <Eigen/Eigen>

#include "loc_msgs/msg/ins.hpp"
#include "inner_msg/msg_base.h"
#include "utils/time_stamp.h"
#include "utils/constant.h"
#include "utils/matrix_utils.h"

namespace Fusion {
using InsMsg = loc_msgs::msg::INS;
using InsMsgPtr = InsMsg::SharedPtr;

struct Ins : MsgBase {
public:
    using Ptr = std::shared_ptr<Ins>;
    Ins() = default;
    virtual ~Ins() = default;
    Ins(const InsMsgPtr& msgPtr) : MsgBase(msgPtr->header.stamp),
        status(msgPtr->status), fusionFlag(msgPtr->fusion_flag), faultFlag(msgPtr->fault_flag),
        lla(msgPtr->lla.latitude, msgPtr->lla.longitude, msgPtr->lla.altitude),
        eular(msgPtr->attitude.roll, msgPtr->attitude.pitch, msgPtr->attitude.yaw),
        vel(msgPtr->linear_velocity.x, msgPtr->linear_velocity.y, msgPtr->linear_velocity.z),
        transAccu(msgPtr->pose_accuracy.x, msgPtr->pose_accuracy.y, msgPtr->pose_accuracy.z),
        attAccu(msgPtr->att_accuracy.roll, msgPtr->att_accuracy.pitch, msgPtr->att_accuracy.yaw),
        velAccu(msgPtr->vel_accuracy.x, msgPtr->vel_accuracy.y, msgPtr->vel_accuracy.z) {}
public:
    uint8_t status;
    uint8_t fusionFlag;
    uint8_t faultFlag;
    Eigen::Vector3d lla;
    Eigen::Vector3d eular;
    Eigen::Vector3d vel;
    Eigen::Vector3d transAccu;
    Eigen::Vector3d attAccu;
    Eigen::Vector3d velAccu;
};

}

#endif