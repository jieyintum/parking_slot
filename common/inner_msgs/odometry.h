#ifndef FUSION_ODOMETRY_H_
#define FUSION_ODOMETRY_H_

#include <Eigen/Eigen>
#include "nav_msgs/msg/odometry.hpp"
#include "loc_msgs/msg/odometry.hpp"
#include "inner_msgs/msg_base.h"
#include "utils/matrix_utils.h"

namespace Fusion
{
    using OdometryMsg = loc_msgs::msg::Odometry;
    using OdometryMsgPtr = OdometryMsg::SharedPtr;
    struct Odometry : MsgBase
    {
        using Ptr = std::shared_ptr<Odometry>;
        Odometry() = default;
        ~Odometry() = default;
        Odometry(const OdometryMsgPtr &msg) : MsgBase(msg->header.stamp), status(msg->status), source(msg->source),
                                              translation(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
                                              orientation(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
                                              linearVelocity(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z),
                                              angularVelocity(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z) {}

        std::uint8_t status = 0U;
        std::uint8_t source = 0U;
        Eigen::Vector3d translation{0.0, 0.0, 0.0};
        Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
        Eigen::Vector3d linearVelocity{0.0, 0.0, 0.0};
        Eigen::Vector3d angularVelocity{0.0, 0.0, 0.0};
        Eigen::Vector3d accelerate{0.0, 0.0, 0.0};
        Eigen::Vector3d accBias{0.0, 0.0, 0.0};
        Eigen::Vector3d gyroBias{0.0, 0.0, 0.0};

        void ToOdomMsg(loc_msgs::msg::Odometry &msg) const
        {
            msg.header.stamp = timestamp.ToMsg();
            msg.header.frame_id = "odom";
            msg.childframe_id = "ego";
            msg.status = status;
            msg.source = source;
            msg.pose.position.x = translation.x();
            msg.pose.position.y = translation.y();
            msg.pose.position.z = translation.z();
            msg.pose.orientation.x = orientation.x();
            msg.pose.orientation.y = orientation.y();
            msg.pose.orientation.z = orientation.z();
            msg.pose.orientation.w = orientation.w();
            auto eulerAngle = Quaternion2Eular(orientation);
            eulerAngle = eulerAngle * 180.0 / M_PI;
            msg.pose.covariance[0] = eulerAngle.x();
            msg.pose.covariance[1] = -eulerAngle.y();
            msg.pose.covariance[2] = eulerAngle.z();

            msg.twist.linear.x = linearVelocity.x();
            msg.twist.linear.y = linearVelocity.y();
            msg.twist.linear.z = linearVelocity.z();
            msg.twist.angular.x = angularVelocity.x();
            msg.twist.angular.y = angularVelocity.y();
            msg.twist.angular.z = angularVelocity.z();
            msg.twist.linear_covariance[0] = accBias.x();
            msg.twist.linear_covariance[1] = accBias.y();
            msg.twist.linear_covariance[2] = accBias.z();

            msg.twist.linear_covariance[3] = gyroBias.x();
            msg.twist.linear_covariance[4] = gyroBias.y();
            msg.twist.linear_covariance[5] = gyroBias.z();

            msg.accelaration.accelaration.x = accelerate.x();
            msg.accelaration.accelaration.y = accelerate.y();
            msg.accelaration.accelaration.z = accelerate.z();
        }

        void ToOdomMsg(nav_msgs::msg::Odometry &msg) const
        {
            msg.header.stamp = timestamp.ToMsg();
            msg.header.frame_id = "odom";
            msg.child_frame_id = "ego";
            msg.pose.pose.position.x = translation.x();
            msg.pose.pose.position.y = translation.y();
            msg.pose.pose.position.z = translation.z();
            msg.pose.pose.orientation.x = orientation.x();
            msg.pose.pose.orientation.y = orientation.y();
            msg.pose.pose.orientation.z = orientation.z();
            msg.pose.pose.orientation.w = orientation.w();
            msg.twist.twist.linear.x = linearVelocity.x();
            msg.twist.twist.linear.y = linearVelocity.y();
            msg.twist.twist.linear.z = linearVelocity.z();
            msg.twist.twist.angular.x = angularVelocity.x();
            msg.twist.twist.angular.y = angularVelocity.y();
            msg.twist.twist.angular.z = angularVelocity.z();
            msg.twist.covariance[0] = accBias.x();
            msg.twist.covariance[1] = accBias.y();
            msg.twist.covariance[2] = accBias.z();

            msg.twist.covariance[3] = gyroBias.x();
            msg.twist.covariance[4] = gyroBias.y();
            msg.twist.covariance[5] = gyroBias.z();
        }
    };
}

#endif