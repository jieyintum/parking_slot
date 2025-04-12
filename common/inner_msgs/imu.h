#ifndef FUSION_IMU_H_
#define FUSION_IMU_H_

#include <cstdint>
#include <Eigen/Eigen>

#include "sensor_msgs/msg/imu.h"
#include "loc_msgs/msg/imu.hpp"

#include "utils/time_stamp.h"
#include "utils/matrix_utils.h"

namespace Fusion
{
    using ImuMsg = loc_msgs::msg::IMU; // this can be changed according to msg
    using ImuMsgPtr = ImuMsg::SharedPtr;
    struct Imu
    {
        Imu() = default;
        Imu(const ImuMsgPtr &msgPtr, const bool isTransformToBase = false) : timestamp(msgPtr->header.stamp), status(msgPtr->status)
        {
            const auto &gyroIn = msgPtr->angular_rate;
            gyro << gyroIn.x, gyroIn.y, gyroIn.z;
            const auto &accIn = msgPtr->linear_accelaration;
            acc << accIn.x, accIn.y, accIn.z;
            const auto &installError = msgPtr->install_error;
            installErrorAng << installError.roll, installError.pitch, installError.yaw;
            if (isTransformToBase)
            {
                installErrorAng.y() = std::max(installErrorAng.y(), -0.3);
                installErrorAng.y() = std::min(installErrorAng.y(), 0.3);
                const auto rbv = Eular2RotationMatrix(installErrorAng);
                acc = rbv.transpose() * acc;
                gyro = rbv.transpose() * gyro;
            }
        }
        TimeStamp timestamp;
        std::uint8_t status;
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
        Eigen::Vector3d installErrorAng;

        using Ptr = std::shared_ptr<Imu>;
    };
}

#endif