#ifndef FUSION_ODOM_BUFFER_H_
#define FUSION_ODOM_BUFFER_H_

#include "inner_msgs/odometry.h"
#include "utils/time_fixed_size_deque.h"

namespace Fusion {
class OdomBuffer : public TimeFixedSizeDeque<Odometry::Ptr> {
public:
    static OdomBuffer& GetInstance() {
        static OdomBuffer instance(500U);
        return instance;
    }

public:
    bool GetDeltaPose(const TimeStamp& stamp1, const TimeStamp& stamp2, Odometry& deltaPose)
    {
        auto odom1 = GetInterpolation(stamp1);
        auto odom2 = GetInterpolation(stamp2);
        if ((odom1 != nullptr) && (odom2 != nullptr)) {
            deltaPose.translation = odom1->orientation.inverse() * (odom2->translation - odom1->translation);
            deltaPose.orientation = odom1->orientation.inverse() * odom2->orientation;
        }
        return (odom1 != nullptr) && (odom2 != nullptr);
    }

private:
    OdomBuffer(const std::uint16_t fixedSize) : TimeFixedSizeDeque<Odometry::Ptr>(fixedSize) {}
    OdomBuffer(OdomBuffer&) = delete;
    OdomBuffer& operator= (const OdomBuffer&) = delete;

private:
    Odometry::Ptr Interpolation(const Odometry::Ptr& msg1, const Odometry::Ptr& msg2, const TimeStamp& timestamp) override
    {
        if (msg1->GetTime() == timestamp) {
            return msg1;
        }
            
        if (msg2->GetTime() == timestamp) {
            return msg2;
        }

        Odometry::Ptr res = std::make_shared<Odometry>();
        const double t1 = msg1->GetTime().Sec();
        const double t2 = msg2->GetTime().Sec();
        const double t = timestamp.Sec();
        const double alpha = (t - t1) / (t2 - t1);
        res->timestamp = timestamp;
        res->translation = msg1->translation + alpha * (msg2->translation - msg1->translation);

        const Eigen::Quaterniond quat01 = msg1->orientation.inverse() * msg2->orientation; // can be optimized
        const auto atti = Quaternion2Eular(quat01);
        res->orientation = msg1->orientation * Eular2Quaternion(alpha * atti);
        res->linearVelocity = msg1->linearVelocity + alpha * (msg2->linearVelocity - msg1->linearVelocity);
        res->angularVelocity = msg1->angularVelocity + alpha * (msg2->angularVelocity - msg1->angularVelocity);

        return res;
    }
};
}
#endif