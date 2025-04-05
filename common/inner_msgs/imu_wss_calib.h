#ifndef FUSION_IMU_WSS_CALIB_H_
#define FUSION_IMU_WSS_CALIB_H_

#include <Eigen/Eigen>
#include "loc_msgs/msg/imu_wss_calib.hpp"

namespace Fusion {
using ImuWssCalibMsg = loc_msgs::msg::ImuWssCalib;
using ImuWssCalibMsgPtr = ImuWssCalibMsg::SharedPtr;

struct ImuWssCalib {
public:
    ImuWssCalib(const ImuWssCalibMsgPtr& msg)
        : status(msg->status), source(msg->source), faultFlag(msg->fault_flag),
          accBias(msg->acc_bias.x, msg->acc_bias.y, msg->acc_bias.z),
          accBiasStd(msg->acc_bias_std.x, msg->acc_bias_std.y, msg->acc_bias_std.z),
          gyroBias(msg->gyro_bias.x, msg->gyro_bias.z, msg->gyro_bias.z),
          gyroBiasStd(msg->gyro_bias_std.x, msg->gyro_bias_std.y, msg->gyro_bias_std.z),
          imuInstallError(msg->imu_install_error.roll, msg->imu_install_error.pitch, msg->imu_install_error.yaw),
          imuInstallErrorStd(msg->imu_install_error_std.roll, msg->imu_install_error_std.pitch, msg->imu_install_error_std.yaw)
    {
        if (!msg->wss_scale.empty()) {
            wssScale.reserve(msg->wss_scale.size());
            for (auto scale : msg->wss_scale) {
                wssScale.push_back(scale);
            }
        }
        if (!msg->wss_scale_std.empty()) {
            wssScaleStd.reserve(msg->wss_scale_std.size());
            for (auto std : msg->wss_scale_std) {
                wssScaleStd.push_back(std);
            }
        }
    }
    std::uint8_t status;
    std::uint8_t source;
    std::uint8_t faultFlag;
    Eigen::Vector3d accBias;
    Eigen::Vector3d accBiasStd;
    Eigen::Vector3d gyroBias;
    Eigen::Vector3d gyroBiasStd;
    Eigen::Vector3d imuInstallError;
    Eigen::Vector3d imuInstallErrorStd;
    std::vector<double> wssScale;
    std::vector<double> wssScaleStd;
};
}

#endif